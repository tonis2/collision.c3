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

<*
 A pool that is never destroyed must not capture the next pool built where it
 stood.

 **This is the regression for a hang, so read what it does carefully.** The
 sequence below is exactly what C3's test runner produces: `test::@check`
 failing calls `longjmp`, which unwinds past every `defer`, so a
 `PhysicsWorld` whose `free()` sat in one is never freed and its workers are
 never joined. The frame is then reused by whatever runs next, and the next
 world initializes its own pool over the abandoned one's bytes.

 When the pool's mutex, condition variables and queue counters lived in the
 handle, that second `init` re-initialized the very objects the stale worker
 was parked on. Two workers then serviced one queue whose `in_flight` was
 accounted for one, and `join()` below never returned — so a single red
 physics check turned the rest of the suite into a hang that named neither the
 check nor the cause.

 `abandoned` stands in for the handle the longjmp threw away, and is kept only
 so this test can clean up after itself; the code under test never sees it.
 **Injecting the bug makes this hang rather than fail**, which is the nature
 of what it guards and the reason it is worth having.
*>
fn void an_abandoned_pool_does_not_capture_the_next_one() @test
{
	WorkerPool slot;
	slot.init(1)!!;

	// Run work through it and join, so the worker is parked on `work_ready`
	// inside the state rather than caught mid-task.
	int first;
	slot.push(&collision_testutil::slot_worker, &first);
	slot.join();
	test::@check(first == 1, "the first pool ran its work: %d", first);

	// The longjmp: `destroy` is skipped and the handle is gone with the frame.
	WorkerPool abandoned = slot;
	slot = { };

	// The next world lands on the same memory and builds its own pool there.
	slot.init(1)!!;
	defer
	{
		slot.destroy();
		abandoned.destroy();
	}

	int second;
	slot.push(&collision_testutil::slot_worker, &second);
	slot.join(); // Hangs here when the stale worker shares this pool's state
	test::@check(second == 1, "the second pool ran its own work: %d", second);
}
