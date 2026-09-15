"""Responsive, serialized UI jobs; does not change any mechanical calculation."""

import asyncio
import time


class SupersededJob(Exception):
    """A newer button request replaces this job at the next safe boundary."""


class ShellJobControls:
    def init_job_controls(self):
        self.work = None
        self.pending_job = None
        self.compute_lock = asyncio.Lock()
        self.job_label = "Ready"
        self.job_started = 0.0
        self.job_step = 0
        self.job_steps = 0
        self.job_outcome = "READY"

    def launch(self, coroutine, label="Calculation"):
        if self.work and not self.work.done():
            if self.pending_job:
                self.pending_job[0].close()
            self.pending_job = (coroutine, label)
            # A paused old job must reach its safe boundary to be superseded.
            self.running = True
            self.feedback.text = "Switching to "+label+" after the current solver step. No extra click needed."
            return
        self.running = True
        self.job_label, self.job_started = label, time.perf_counter()
        self.job_step, self.job_steps, self.job_outcome = 0, 0, "RUNNING"
        self.feedback.text = "Starting "+label+"..."
        self.work = asyncio.ensure_future(self.guard_job(coroutine))
        # Cancellation can arrive before guard_job has first awaited its input.
        self.work.add_done_callback(lambda _task: coroutine.close())

    async def guard_job(self, coroutine):
        try:
            await coroutine
            if self.work is asyncio.current_task() and self.job_outcome == "RUNNING":
                self.job_outcome = "COMPLETE"
        except SupersededJob:
            if self.work is asyncio.current_task():
                self.job_outcome = "SWITCHING"
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            if self.work is asyncio.current_task():
                self.job_outcome = "NOT APPLIED"
                self.feedback.text = "NOT APPLIED: "+str(exc)
        finally:
            # Cancelled workers may drain after a new button request. They must
            # not pause, relabel or launch work into the newer controller job.
            if self.work is asyncio.current_task():
                self.running = False
                next_job, self.pending_job = self.pending_job, None
                if next_job is not None:
                    self.work = None
                    self.launch(*next_job)

    async def wait_running(self):
        while True:
            if self.closed:
                raise asyncio.CancelledError()
            if not self.job_context_valid():
                raise RuntimeError("Scene disconnected. Click Reconnect opened knee; the old result was not applied.")
            if self.pending_job is not None:
                raise SupersededJob()
            if self.running:
                return
            await asyncio.sleep(.02)

    def job_context_valid(self):
        """Override for scene-bound jobs; checked before and after each worker."""
        return True

    async def compute(self, function, *args, **kwargs):
        """At most one numerical worker, even across Neutral/cancel/restart."""
        await self.wait_running()
        async with self.compute_lock:
            await self.wait_running()
            worker = asyncio.ensure_future(asyncio.to_thread(function, *args, **kwargs))
            try:
                result = await asyncio.shield(worker)
            except asyncio.CancelledError:
                # to_thread cannot be killed. Hold the lock until it drains,
                # then discard its candidate instead of applying stale state.
                try:
                    await worker
                finally:
                    raise asyncio.CancelledError()
        await self.wait_running()
        return result

    def cancel_jobs(self):
        if self.pending_job is not None:
            self.pending_job[0].close()
            self.pending_job = None
        if self.work and not self.work.done():
            self.work.cancel()
        self.work = None
        self.running = False
        self.job_outcome = "CANCELLED / NEUTRAL"

    def pause_jobs(self):
        if self.work and not self.work.done():
            self.running = not self.running
            self.feedback.text = ("Resumed; waiting for the next accepted solver step." if self.running
                                  else "PAUSED: geometry held. Press Pause / resume to continue, or choose another movement.")
        else:
            self.feedback.text = "No calculation is running. Choose a cable movement to start."

    def progress_text(self):
        if self.work and not self.work.done():
            if self.pending_job:
                return "SWITCHING to "+self.pending_job[1]+" | finishing current step"
            state = "SOLVING" if self.running else "PAUSED"
            steps = f" | {self.job_step}/{self.job_steps} accepted" if self.job_steps else ""
            return f"{state}: {self.job_label}{steps} | {time.perf_counter()-self.job_started:.0f} s"
        return f"{self.job_outcome} | {self.job_label}"
