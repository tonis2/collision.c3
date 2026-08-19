<*
 The worker pool on its own: that it starts the workers it was asked for, that
 pushing more tasks than the queue holds blocks rather than drops, and that a
 pool survives being joined and reused.

	`c3c test --test-filter collision_tests::pool`

 Kept apart from `collision_tests::world` because the leak report cannot be read
 the same way here. The tracking allocator the runner installs is *thread-local*
 — it replaces `allocator::thread_allocator` for the thread running the test —
 so anything a worker allocates on its own thread is invisible to it. A pool
 leak therefore shows up as a growing RSS and not as a failing test, and this
 file's job is the ownership it can check: that `destroy()` is reached on every
 path and that a reused pool does not stack up state.
*>
module collision_tests::pool;

import collision;
import collision_testutil;
import std::core::test;

fn void test_worker_pool() @test
{
	WorkerPool pool;
	pool.init(3)!!;
	defer pool.destroy();
	test::@check(pool.workers() == 3);

	// More tasks than queue capacity (workers * 4) to exercise blocking push,
	// several rounds to exercise join + reuse
	int[64] slots;
	for (int round = 0; round < 4; round++)
	{
		for (int i = 0; i < 64; i++) slots[i] = 0;
		for (int i = 0; i < 64; i++) pool.push(&collision_testutil::slot_worker, &slots[i]);
		pool.join();
		for (int i = 0; i < 64; i++) test::@check(slots[i] == 1);
	}
}
