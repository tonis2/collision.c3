module collision;

import std::math;
import std::thread;
import std::thread::cpu;

// Runtime-sized worker pool for the physics solver.
//
// The stdlib ThreadPool takes its size as a compile-time template parameter,
// so the worker count could never match the user's actual CPU. This pool is
// sized at init time (defaulting to the machine's core count) and its push()
// blocks on a condition variable when the queue is full, where the stdlib
// pool spin-locks.

// Upper bound on workers — also sizes the solver's fixed per-worker scratch
// arrays (e.g. island buckets), so it must stay a compile-time constant.
const sz MAX_WORKER_THREADS = 32;

<*
 Worker count used when PhysicsWorld.thread_count is 0: all cores minus one
 for the main thread, clamped to [1, MAX_WORKER_THREADS].
*>
fn sz default_worker_count()
{
	return (sz)math::clamp(cpu::native_cpu() - 1, 1, (int)MAX_WORKER_THREADS);
}

struct WorkItem @local
{
	ThreadFn func;
	void* arg;
}

<*
 Everything the workers touch, in one heap block they co-own.

 **This is deliberately not inlined into `WorkerPool`, and the reason is a
 hang.** A worker holds a pointer to this block for its whole life, so the
 block has to outlive the pool handle rather than share its storage. When the
 state lived in the handle, a `PhysicsWorld` that was never freed left its
 workers parked on a mutex inside the *caller's stack frame*; the next world
 built in that frame initialized its own pool over the same bytes, and the
 stale worker woke up sharing another pool's queue, its counters and its
 condition variables. Two workers then serviced a queue accounted for one and
 `join()` never saw it drain.

 That is not a hypothetical. C3's test runner `longjmp`s out of a failing
 check, which skips every `defer` and so skips `PhysicsWorld.free`, so *any*
 red physics check left a worker behind — and the next physics check in the
 same binary hung instead of running. A hang reports nothing and blames
 nobody, which is a far worse failure than the leak it was hiding.

 With the state on the heap, a pool that never reaches `destroy` leaks one
 block and one parked thread and touches nothing else. The leak is still a
 bug; it is just a bug that stays where it was made.

 The same indirection makes the handle safe to move. `create_world` returns a
 `PhysicsWorld` by value, and copying a handle now copies a pointer rather
 than a set of pthread objects the workers are already blocked on. Ownership
 is still single — the moved-from handle must not be destroyed as well.
*>
struct PoolState @local
{
	Mutex mutex;
	ConditionVariable work_ready; // Signaled when work is queued or the pool stops
	ConditionVariable work_done; // Signaled when queue and in-flight work drain
	ConditionVariable space_free; // Signaled when a queue slot frees up
	Thread[] threads;
	WorkItem[] queue; // Ring buffer
	sz queue_head;
	sz queue_len;
	sz in_flight; // Items currently being executed by workers
	sz spawned; // Threads successfully created (for partial-init cleanup)
	bool stop;
}

struct WorkerPool
{
	PoolState* state;
	bool initialized;
}

fn sz WorkerPool.workers(&self)
{
	// Zero before init, which is what the bare `threads.len` answered before
	// the state moved off the handle — the solver divides by this and reads it
	// on paths that run before `run_step` has lazily initialized anything.
	if (self.state == null) return 0;
	return self.state.threads.len;
}

<*
 @require !self.initialized : `WorkerPool must not be already initialized`
 @require workers > 0 : `Need at least one worker`
*>
fn void? WorkerPool.init(&self, sz workers)
{
	workers = math::min(workers, MAX_WORKER_THREADS);

	*self = { .state = mem::new(PoolState), .initialized = true };
	defer catch self.destroy();

	PoolState* state = self.state;
	state.mutex.init()!;
	state.work_ready.init()!;
	state.work_done.init()!;
	state.space_free.init()!;
	state.threads = mem::new_array(Thread, workers);
	state.queue = mem::new_array(WorkItem, workers * 4);

	foreach (&thread : state.threads)
	{
		thread.create(&worker_main, state)!;
		state.spawned++;
	}
}

fn int worker_main(void* arg) @local
{
	PoolState* pool = (PoolState*)arg;

	while (true)
	{
		pool.mutex.lock();
		while (pool.queue_len == 0 && !pool.stop)
		{
			pool.work_ready.wait(&pool.mutex);
		}
		if (pool.queue_len == 0)
		{
			// stop requested and the queue is drained
			pool.mutex.unlock();
			return 0;
		}

		WorkItem item = pool.queue[pool.queue_head];
		pool.queue_head = (pool.queue_head + 1) % pool.queue.len;
		pool.queue_len--;
		pool.in_flight++;
		pool.space_free.signal();
		pool.mutex.unlock();

		item.func(item.arg);

		pool.mutex.lock();
		pool.in_flight--;
		if (pool.queue_len == 0 && pool.in_flight == 0)
		{
			pool.work_done.broadcast();
		}
		pool.mutex.unlock();
	}
}

<*
 Queue a job. Blocks (without spinning) while the queue is full.
*>
fn void WorkerPool.push(&self, ThreadFn func, void* arg)
{
	PoolState* pool = self.state;

	pool.mutex.lock();
	while (pool.queue_len == pool.queue.len)
	{
		pool.space_free.wait(&pool.mutex);
	}
	sz tail = (pool.queue_head + pool.queue_len) % pool.queue.len;
	pool.queue[tail] = { .func = func, .arg = arg };
	pool.queue_len++;
	pool.work_ready.signal();
	pool.mutex.unlock();
}

<*
 Wait until all queued and in-flight work has completed.
*>
fn void WorkerPool.join(&self)
{
	if (!self.initialized) return;

	PoolState* pool = self.state;

	pool.mutex.lock();
	while (pool.queue_len > 0 || pool.in_flight > 0)
	{
		pool.work_done.wait(&pool.mutex);
	}
	pool.mutex.unlock();
}

<*
 Stop the workers (remaining queued work is still processed), join them,
 and release all resources. Safe to call on a partially initialized pool.
*>
fn void WorkerPool.destroy(&self)
{
	if (!self.initialized) return;

	PoolState* pool = self.state;
	if (pool == null)
	{
		*self = { };
		return;
	}

	if (pool.spawned > 0)
	{
		pool.mutex.lock();
		pool.stop = true;
		pool.work_ready.broadcast();
		pool.mutex.unlock();

		foreach (&thread : pool.threads[:pool.spawned])
		{
			thread.join();
		}
	}

	pool.mutex.destroy();
	pool.work_ready.destroy();
	pool.work_done.destroy();
	pool.space_free.destroy();
	if (pool.threads.ptr != null) mem::free(pool.threads.ptr);
	if (pool.queue.ptr != null) mem::free(pool.queue.ptr);
	// Last, and only after every worker has been joined: nothing may be
	// dereferencing this block once it goes back to the allocator.
	mem::free(pool);
	*self = { };
}
