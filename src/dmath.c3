module collision::dmath;

// Deterministic transcendental math for lockstep simulation.
//
// libm's sin/cos/atan2/pow give different last-bit results per platform and
// libc version, which desyncs lockstep netcode. These implementations use
// only IEEE-754 add/sub/mul/div and integer bit manipulation — operations
// that are correctly rounded and bit-identical on every IEEE platform — with
// double-precision internals so the float results are accurate to well under
// a float ulp.
//
// Requires strict FP codegen (c3c -O3 or below, or an explicit
// --fp-math=strict): relaxed/fast math may contract or reassociate the
// kernels and break both accuracy and cross-platform bit-equality.
//
// Kernels and coefficients follow fdlibm (Sun Microsystems, freely
// distributable), the same source musl and most libms derive from.

// Adding then subtracting this rounds a double to the nearest integer
// (round-half-even) using two exact IEEE additions. Valid for |x| < 2^51.
const double RND_MAGIC @local = 6755399441055744.0; // 1.5 * 2^52

const double PI_D @local = 3.14159265358979311600e+00;
const double PIO2_D @local = 1.57079632679489655800e+00;
const double INV_PIO2 @local = 6.36619772367581382433e-01; // 2/pi
const double PIO2_HI @local = 1.57079632673412561417e+00; // pi/2 high 33 bits
const double PIO2_LO @local = 6.07710050650619224932e-11; // pi/2 - PIO2_HI
const double LN2_D @local = 6.93147180559945286227e-01;
const double INV_LN2 @local = 1.44269504088896338700e+00;

// sin(r) polynomial on [-pi/4, pi/4] (fdlibm k_sin)
const double S1 @local = -1.66666666666666324348e-01;
const double S2 @local = 8.33333333332248946124e-03;
const double S3 @local = -1.98412698298579493134e-04;
const double S4 @local = 2.75573137070700676789e-06;
const double S5 @local = -2.50507602534068634195e-08;
const double S6 @local = 1.58969099521155010221e-10;

// cos(r) polynomial on [-pi/4, pi/4] (fdlibm k_cos)
const double C1 @local = 4.16666666666666019037e-02;
const double C2 @local = -1.38888888888741095749e-03;
const double C3 @local = 2.48015872894767294178e-05;
const double C4 @local = -2.75573143513906633035e-07;
const double C5 @local = 2.08757232129817482790e-09;
const double C6 @local = -1.13596475577881948265e-11;

fn double k_sin(double r) @local
{
	double z = r * r;
	return r + r * z * (S1 + z * (S2 + z * (S3 + z * (S4 + z * (S5 + z * S6)))));
}

fn double k_cos(double r) @local
{
	double z = r * r;
	double w = 1.0 - 0.5 * z;
	return w + z * z * (C1 + z * (C2 + z * (C3 + z * (C4 + z * (C5 + z * C6)))));
}

// Reduce x to r in [-pi/4, pi/4] with x = r + quadrant * pi/2.
// Two-term Cody-Waite: accurate to double precision for |x| up to ~2^20,
// far beyond any angle the solver produces.
fn double reduce_pio2(double x, long* quadrant) @local
{
	double fl = (x * INV_PIO2 + RND_MAGIC) - RND_MAGIC;
	*quadrant = (long)fl;
	return (x - fl * PIO2_HI) - fl * PIO2_LO;
}

<*
 Deterministic sin. Bit-identical on every IEEE 754 platform under strict FP.
*>
fn float sin(float x)
{
	long q;
	double r = reduce_pio2((double)x, &q);
	switch (q & 3)
	{
		case 0: return (float)k_sin(r);
		case 1: return (float)k_cos(r);
		case 2: return (float)-k_sin(r);
		default: return (float)-k_cos(r);
	}
}

<*
 Deterministic cos. Bit-identical on every IEEE 754 platform under strict FP.
*>
fn float cos(float x)
{
	long q;
	double r = reduce_pio2((double)x, &q);
	switch (q & 3)
	{
		case 0: return (float)k_cos(r);
		case 1: return (float)-k_sin(r);
		case 2: return (float)-k_cos(r);
		default: return (float)k_sin(r);
	}
}

<*
 Deterministic sin and cos of the same angle, sharing one range reduction.
 @return "{sin(x), cos(x)}"
*>
fn float[<2>] sincos(float x)
{
	long q;
	double r = reduce_pio2((double)x, &q);
	double s = k_sin(r);
	double c = k_cos(r);
	switch (q & 3)
	{
		case 0: return { (float)s, (float)c };
		case 1: return { (float)c, (float)-s };
		case 2: return { (float)-s, (float)-c };
		default: return { (float)-c, (float)s };
	}
}

// atan polynomial and split high/low interval constants (fdlibm s_atan)
const double[4] ATAN_HI @local = {
	4.63647609000806093515e-01, // atan(0.5)
	7.85398163397448278999e-01, // atan(1.0)
	9.82793723247329054082e-01, // atan(1.5)
	1.57079632679489655800e+00, // atan(inf)
};
const double[4] ATAN_LO @local = {
	2.26987774529616870924e-17,
	3.06161699786838301793e-17,
	1.39033110312309984516e-17,
	6.12323399573676603587e-17,
};
const double AT0 @local = 3.33333333333329318027e-01;
const double AT1 @local = -1.99999999998764832476e-01;
const double AT2 @local = 1.42857142725034663711e-01;
const double AT3 @local = -1.11111104054623557880e-01;
const double AT4 @local = 9.09088713343650656196e-02;
const double AT5 @local = -7.69187620504482999495e-02;
const double AT6 @local = 6.66107313738753120669e-02;
const double AT7 @local = -5.83357013379057348645e-02;
const double AT8 @local = 4.97687799461593236017e-02;
const double AT9 @local = -3.65315727442169155270e-02;
const double AT10 @local = 1.62858201153657823623e-02;

fn double atan_d(double x) @local
{
	bool neg = x < 0;
	double ax = neg ? -x : x;
	int id;

	switch
	{
		case ax < 0.4375:
			if (ax < 7.4505805969238281e-09) return x; // < 2^-27: atan(x) ~ x
			id = -1;
		case ax < 0.6875:
			id = 0;
			ax = (2.0 * ax - 1.0) / (2.0 + ax);
		case ax < 1.1875:
			id = 1;
			ax = (ax - 1.0) / (ax + 1.0);
		case ax < 2.4375:
			id = 2;
			ax = (ax - 1.5) / (1.0 + 1.5 * ax);
		case ax < 7.3786976294838206e+19: // 2^66
			id = 3;
			ax = -1.0 / ax;
		default:
			return neg ? -(ATAN_HI[3] + ATAN_LO[3]) : ATAN_HI[3] + ATAN_LO[3];
	}

	double z = ax * ax;
	double w = z * z;
	double s1 = z * (AT0 + w * (AT2 + w * (AT4 + w * (AT6 + w * (AT8 + w * AT10)))));
	double s2 = w * (AT1 + w * (AT3 + w * (AT5 + w * (AT7 + w * AT9))));

	if (id < 0)
	{
		double r = ax - ax * (s1 + s2);
		return neg ? -r : r;
	}
	double r = ATAN_HI[id] - ((ax * (s1 + s2) - ATAN_LO[id]) - ax);
	return neg ? -r : r;
}

<*
 Deterministic atan.
*>
fn float atan(float x) => (float)atan_d((double)x);

<*
 Deterministic atan2. Quadrant conventions match libm for nonzero inputs and
 signed zero y; atan2(±0, ±0) returns ±0 (libm returns ±0 or ±pi depending on
 the sign of x).
*>
fn float atan2(float y, float x)
{
	if (x == 0 && y == 0) return y;
	bool y_neg = (bitcast(y, uint) >> 31) != 0;
	if (x == 0) return y_neg ? (float)-PIO2_D : (float)PIO2_D;

	double r = atan_d((double)y / (double)x);
	if (x > 0) return (float)r;
	return (float)(y_neg ? r - PI_D : r + PI_D);
}

// log2 via atanh series: with m in [sqrt(2)/2, sqrt(2)) and s = (m-1)/(m+1),
// ln(m) = s * (2 + (2/3)s^2 + (2/5)s^4 + ...). |s| <= 0.172 so seven terms
// reach ~1e-12 relative error.
fn double log2_d(double x) @local
{
	ulong bits = bitcast(x, ulong);
	long e = (long)((bits >> 52) & 0x7ff);
	if (e == 0)
	{
		// Subnormal input: renormalize with an exact power-of-two scale
		x *= 18014398509481984.0; // 2^54
		bits = bitcast(x, ulong);
		e = (long)((bits >> 52) & 0x7ff) - 54;
	}
	e -= 1023;
	bits = (bits & 0x000f_ffff_ffff_ffff) | 0x3ff0_0000_0000_0000;
	double m = bitcast(bits, double);
	if (m > 1.4142135623730951)
	{
		m *= 0.5;
		e += 1;
	}

	double s = (m - 1.0) / (m + 1.0);
	double z = s * s;
	double p = 2.0 + z * (0.66666666666666666 + z * (0.4 + z * (0.2857142857142857
		+ z * (0.2222222222222222 + z * (0.18181818181818182 + z * 0.15384615384615385)))));
	return (double)e + s * p * INV_LN2;
}

// 2^y for y in (-1075, 1024): split into integer k and fraction r, evaluate
// exp(r*ln2) by Taylor series (|r*ln2| <= 0.347, ten terms reach ~1e-12),
// scale by 2^k through direct exponent-bit construction.
fn double exp2_d(double y) @local
{
	double fl = (y + RND_MAGIC) - RND_MAGIC;
	long k = (long)fl;
	if (k > 1023) return bitcast((ulong)0x7ff0_0000_0000_0000, double); // +inf
	if (k < -1022) return 0.0; // subnormal double, zero as float

	double t = (y - fl) * LN2_D;
	double p = 1.0 + t * (1.0 + t * (0.5 + t * (0.16666666666666666
		+ t * (0.041666666666666664 + t * (0.008333333333333333
		+ t * (0.001388888888888889 + t * (0.0001984126984126984
		+ t * 2.48015873015873e-05)))))));

	double scale = bitcast((ulong)(k + 1023) << 52, double);
	return p * scale;
}

<*
 Deterministic pow for a non-negative base (2^(exponent*log2(base))).
 Accurate to a few float ulps; bit-identical on every IEEE platform.
 @require base >= 0 : `dmath::pow requires a non-negative base`
*>
fn float pow(float base, float exponent)
{
	if (exponent == 0) return 1.0f;
	if (base == 0) return 0.0f;
	return (float)exp2_d((double)exponent * log2_d((double)base));
}
