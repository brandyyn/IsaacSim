"""Controller concurrency checks without Kit or any mechanical approximation."""

import asyncio
import threading
import unittest
from types import SimpleNamespace

from exact_joint.shell_ui_jobs import ShellJobControls


class Controller(ShellJobControls):
    def __init__(self):
        self.closed = False
        self.feedback = SimpleNamespace(text="")
        self.running = False
        self.published = []
        self.init_job_controls()

    async def job(self, function):
        result = await self.compute(function)
        self.published.append(result)


class JobTests(unittest.IsolatedAsyncioTestCase):
    async def blocked_worker(self):
        started = asyncio.Event()
        release = threading.Event()
        self.addCleanup(release.set)
        loop = asyncio.get_running_loop()

        def worker():
            loop.call_soon_threadsafe(started.set)
            if not release.wait(5):
                raise TimeoutError("Test did not release worker")
            return "old"

        return started, release, worker

    async def finish(self, controller):
        for _ in range(4):
            task = controller.work
            if task:
                await asyncio.wait_for(asyncio.shield(task), 3)
            if controller.work is task:
                return
        self.fail("Controller never became idle")

    async def test_last_button_wins_and_old_candidate_is_discarded(self):
        c = Controller()
        started, release, worker = await self.blocked_worker()
        c.launch(c.job(worker), "old")
        await started.wait()
        c.launch(c.job(lambda: "middle"), "middle")
        c.launch(c.job(lambda: "new"), "new")
        self.assertIn("SWITCHING to new", c.progress_text())
        release.set()
        await self.finish(c)
        self.assertEqual(c.published, ["new"])
        self.assertEqual(c.job_label, "new")
        self.assertEqual(c.job_outcome, "COMPLETE")

    async def test_neutral_then_restart_serializes_and_does_not_publish_old(self):
        c = Controller()
        started, release, worker = await self.blocked_worker()
        c.launch(c.job(worker), "old")
        await started.wait()
        old_task = c.work
        c.cancel_jobs()
        new_started = threading.Event()

        def new_worker():
            new_started.set()
            return "new"

        c.launch(c.job(new_worker), "new")
        await asyncio.sleep(.03)
        self.assertFalse(new_started.is_set(), "Old worker must drain before new work")
        release.set()
        await self.finish(c)
        await old_task
        self.assertEqual(c.published, ["new"])
        self.assertEqual(c.job_outcome, "COMPLETE")

    async def test_pause_holds_geometry_then_resumes(self):
        c = Controller()
        started, release, worker = await self.blocked_worker()
        c.launch(c.job(worker), "motion")
        await started.wait()
        c.pause_jobs()
        release.set()
        await asyncio.sleep(.03)
        self.assertEqual(c.published, [])
        self.assertIn("PAUSED", c.progress_text())
        c.pause_jobs()
        await self.finish(c)
        self.assertEqual(c.published, ["old"])

    async def test_new_button_replaces_paused_job(self):
        c = Controller()
        started, release, worker = await self.blocked_worker()
        c.launch(c.job(worker), "old")
        await started.wait()
        c.pause_jobs()
        c.launch(c.job(lambda: "new"), "new")
        release.set()
        await self.finish(c)
        self.assertEqual(c.published, ["new"])

    async def test_exception_is_visible_and_not_complete(self):
        c = Controller()

        def invalid():
            raise ValueError("Cable tension must be 0-10 N")

        c.launch(c.job(invalid), "invalid")
        await self.finish(c)
        self.assertEqual(c.job_outcome, "NOT APPLIED")
        self.assertIn("0-10 N", c.feedback.text)
        self.assertEqual(c.published, [])

    async def test_guard_stop_is_not_relabelled_complete(self):
        c = Controller()

        async def rejected():
            c.job_outcome = "STOPPED: numerical guard"

        c.launch(rejected(), "guarded")
        await self.finish(c)
        self.assertEqual(c.job_outcome, "STOPPED: numerical guard")

    async def test_cancel_before_start_closes_input_coroutine(self):
        c = Controller()
        coroutine = c.job(lambda: "unused")
        c.launch(coroutine, "unused")
        task = c.work
        c.cancel_jobs()
        await asyncio.gather(task, return_exceptions=True)
        self.assertIsNone(coroutine.cr_frame)
        self.assertEqual(c.published, [])

    async def test_external_task_cancellation_clears_running_state(self):
        c = Controller()

        async def waiting_job():
            await asyncio.sleep(60)

        c.launch(waiting_job(), "cancelled")
        task = c.work
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        self.assertFalse(c.running)
        self.assertEqual(c.job_outcome, "CANCELLED")
        self.assertIn("choose a movement", c.feedback.text)

    async def test_scene_change_while_worker_runs_never_publishes_candidate(self):
        c = Controller()
        c.valid = True
        c.job_context_valid = lambda: c.valid
        started, release, worker = await self.blocked_worker()
        c.launch(c.job(worker), "motion")
        await started.wait()
        c.valid = False
        release.set()
        await self.finish(c)
        self.assertEqual(c.published, [])
        self.assertEqual(c.job_outcome, "NOT APPLIED")
        self.assertIn("Reconnect opened knee", c.feedback.text)

    async def test_disconnected_context_never_starts_worker(self):
        c = Controller()
        c.job_context_valid = lambda: False
        started = []
        c.launch(c.job(lambda: started.append(True)), "motion")
        await self.finish(c)
        self.assertEqual(started, [])
        self.assertEqual(c.published, [])


if __name__ == "__main__":
    unittest.main()
