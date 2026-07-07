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

struct WorkerPool
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
	bool initialized;
}

fn sz WorkerPool.workers(&self) => self.threads.len;

<*
 @require !self.initialized : `WorkerPool must not be already initialized`
 @require workers > 0 : `Need at least one worker`
*>
fn void? WorkerPool.init(&self, sz workers)
{
	workers = math::min(workers, MAX_WORKER_THREADS);

	*self = { .initialized = true };
	defer catch self.destroy();

	self.mutex.init()!;
	self.work_ready.init()!;
	self.work_done.init()!;
	self.space_free.init()!;
	self.threads = mem::new_array(Thread, workers);
	self.queue = mem::new_array(WorkItem, workers * 4);

	foreach (&thread : self.threads)
	{
		thread.create(&worker_main, self)!;
		self.spawned++;
	}
}

fn int worker_main(void* arg) @local
{
	WorkerPool* pool = (WorkerPool*)arg;

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
	self.mutex.lock();
	while (self.queue_len == self.queue.len)
	{
		self.space_free.wait(&self.mutex);
	}
	sz tail = (self.queue_head + self.queue_len) % self.queue.len;
	self.queue[tail] = { .func = func, .arg = arg };
	self.queue_len++;
	self.work_ready.signal();
	self.mutex.unlock();
}

<*
 Wait until all queued and in-flight work has completed.
*>
fn void WorkerPool.join(&self)
{
	if (!self.initialized) return;

	self.mutex.lock();
	while (self.queue_len > 0 || self.in_flight > 0)
	{
		self.work_done.wait(&self.mutex);
	}
	self.mutex.unlock();
}

<*
 Stop the workers (remaining queued work is still processed), join them,
 and release all resources. Safe to call on a partially initialized pool.
*>
fn void WorkerPool.destroy(&self)
{
	if (!self.initialized) return;

	if (self.spawned > 0)
	{
		self.mutex.lock();
		self.stop = true;
		self.work_ready.broadcast();
		self.mutex.unlock();

		foreach (&thread : self.threads[:self.spawned])
		{
			thread.join();
		}
	}

	self.mutex.destroy();
	self.work_ready.destroy();
	self.work_done.destroy();
	self.space_free.destroy();
	if (self.threads.ptr != null) mem::free(self.threads.ptr);
	if (self.queue.ptr != null) mem::free(self.queue.ptr);
	*self = { };
}
