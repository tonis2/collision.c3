<*
 The worker pool on its own: that it starts the workers it was asked for, that
 pushing more tasks than the queue holds blocks rather than drops, and that a
 pool survives being joined, reused, destroyed with work still queued, and
 destroyed having never been initialized at all.

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

<*
 `destroy` stops the workers but does not drop what is already queued.

 The solver leans on this: `dispatch_phase_tasks` pushes a phase and joins it,
 but a world torn down mid-frame (an error path, or a caller that frees after
 pushing) must not leave half a phase unexecuted with the other half's results
 already read. The worker loop only exits on `stop` *and* an empty queue, and
 this pins that ordering down.

 Whether any item is still sitting in the queue when `stop` is set is a race —
 the queue holds `workers * 4` and `push` blocks once it is full, so a fast
 worker may well keep up. The assertion does not depend on winning that race:
 every pushed item must have run by the time `destroy` returns, on any
 interleaving.

 Injected — letting the worker return on `stop` with the queue non-empty — this
 goes red with `item 29 ran exactly once: 0`.
*>
fn void destroy_finishes_the_work_still_in_the_queue() @test
{
	WorkerPool pool;
	pool.init(1)!!;

	int[32] slots;
	foreach (i, &slot : slots) pool.push(&collision_testutil::slot_worker, slot);

	// No join(): destroy is the only thing that waits for these.
	pool.destroy();

	foreach (i, slot : slots) test::@check(slot == 1, "item %d ran exactly once: %d", i, slot);
}

<*
 A pool that was never initialized answers rather than dereferences.

 `PhysicsWorld` initializes its pool lazily, inside the first `run_step`, so a
 world that is built and freed without ever being stepped — every test that
 only checks construction, and every early error return — reaches `destroy`
 with a null `state`. The solver also reads `workers()` on paths that run
 before that lazy init to size its per-worker scratch, and needs the zero back
 rather than a crash.

 Injected — the null check taken out of `workers()` — this segfaults rather
 than failing, which is exactly the crash it exists to keep out of the solver.
*>
fn void an_uninitialized_pool_answers_without_touching_its_state() @test
{
	WorkerPool pool;

	test::@check(pool.workers() == 0, "no workers before init: %d", (int)pool.workers());
	pool.join();
	pool.destroy();
	test::@check(pool.workers() == 0, "still none after a no-op destroy: %d", (int)pool.workers());
}

<*
 `destroy` leaves the handle in the state `init` demands, so it can be reused.

 `init` is contracted on `!self.initialized`, which makes "destroy then init
 again" either legal or a contract violation depending on what `destroy`
 leaves behind — and the answer has to be legal, because a `PhysicsWorld` is
 allowed to be freed and rebuilt in the same variable.

 Injected — the trailing `*self = { }` dropped from `destroy` — this trips the
 contract instead of running.
*>
fn void a_destroyed_pool_can_be_initialized_again() @test
{
	WorkerPool pool;
	pool.init(2)!!;
	pool.destroy();

	pool.init(2)!!;
	defer pool.destroy();
	test::@check(pool.workers() == 2, "the second pool has its own workers: %d", (int)pool.workers());

	int slot;
	pool.push(&collision_testutil::slot_worker, &slot);
	pool.join();
	test::@check(slot == 1, "and runs work: %d", slot);
}

<*
 The worker count is clamped, not trusted.

 `MAX_WORKER_THREADS` sizes the solver's fixed per-worker scratch arrays (the
 island buckets), so a `thread_count` above it would have workers indexing past
 the end of those arrays. `init` clamps instead of failing, which means the
 clamp is the only thing standing between a large `thread_count` and an
 out-of-bounds write — worth a check of its own.

 Injected — the clamp removed — this reports 40 workers against 32 buckets.
*>
fn void the_worker_count_is_clamped_to_the_scratch_bound() @test
{
	WorkerPool pool;
	pool.init(collision::MAX_WORKER_THREADS + 8)!!;
	defer pool.destroy();

	test::@check(pool.workers() == collision::MAX_WORKER_THREADS,
		"clamped to the scratch bound: %d", (int)pool.workers());

	int slot;
	pool.push(&collision_testutil::slot_worker, &slot);
	pool.join();
	test::@check(slot == 1, "and the clamped pool still works: %d", slot);
}
