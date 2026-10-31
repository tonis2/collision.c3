module collision;

<*
 One XPBD Gauss-Seidel step for a single constraint: `dl = -(c + alpha*lambda) / (w + alpha)`.
 Adds `dl` to the accumulated multiplier `lambda` and returns it.

 @param c : `Constraint value (violation).`
 @param w : `Generalized inverse mass along the constraint gradient.`
 @param alpha : `Compliance divided by dt squared.`
 @param [&inout] lambda : `Accumulated Lagrange multiplier.`
*>
fn float xpbd_delta_lambda(float c, float w, float alpha, float* lambda)
{
	float dl = -(c + alpha * *lambda) / (w + alpha);
	*lambda += dl;
	return dl;
}

<*
 Progressive stiffness for iteration `iteration` (0-based) of `total`: starts at
 4x `base` compliance and ramps linearly down to `base` on the last iteration.
 A zero `base` (rigid) or a single iteration is returned unchanged.
*>
fn float ramped_compliance(float base, uint iteration, uint total)
{
	if (base > 0 && total > 1)
	{
		float progress = (float)(iteration + 1) / (float)total;
		return base * (1.0f + 3.0f * (1.0f - progress));
	}
	return base;
}

<*
 XPBD compliance for stiffness `k`: `(1 / k) / dt²`, or `fallback` when `k <= 0`.
*>
fn float stiffness_to_compliance(float k, float dt, float fallback) => k > 0 ? (1.0f / k) / (dt * dt) : fallback;
