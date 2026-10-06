"""Bounded lifecycle adapter around the vendored SIYI SDK."""

from __future__ import annotations

import threading
import time
from typing import Any

from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.vision.peripheral.siyi.hardware.ports import SiyiSdkPort


SDK_DISCONNECT_TIMEOUT_S = 2.0


class BoundedSiyiSdk:
    """Forward SDK calls while making its blocking disconnect retryable."""

    def __init__(
        self,
        sdk: SiyiSdkPort,
        timeout_s: float = SDK_DISCONNECT_TIMEOUT_S,
    ) -> None:
        self._sdk = sdk
        self._timeout_s = float(timeout_s)
        self._operation_lock = threading.Lock()
        self._lock = threading.Lock()
        self._disconnect_thread: threading.Thread | None = None
        self._disconnect_launch: ThreadLaunchGate | None = None
        self._disconnect_finished: threading.Event | None = None
        self._disconnect_error: BaseException | None = None
        self._disconnected = False
        self._zero_rate_acknowledged = False

    def __getattr__(self, name: str) -> Any:
        return getattr(self._sdk, name)

    def disconnect(self) -> bool:
        with self._operation_lock:
            return self._disconnect_serial()

    def shutdown_payload(self) -> bool:
        with self._operation_lock:
            if not self._zero_rate_acknowledged:
                acknowledged = self._sdk.requestGimbalSpeed(0, 0)
                if acknowledged is not True:
                    raise RuntimeError(
                        "SIYI zero-rate command was not acknowledged"
                    )
                self._zero_rate_acknowledged = True
            return self._disconnect_serial()

    def _disconnect_serial(self) -> bool:
        deadline_s = time.monotonic() + max(0.0, self._timeout_s)
        with self._lock:
            if self._disconnected:
                return True
            thread = self._disconnect_thread
            if thread is None:
                launch = ThreadLaunchGate()
                finished = threading.Event()
                thread = threading.Thread(
                    target=self._run_disconnect,
                    args=(launch, finished),
                    name="siyi-sdk-disconnect",
                    daemon=True,
                )
                self._disconnect_thread = thread
                self._disconnect_launch = launch
                self._disconnect_finished = finished
                try:
                    thread.start()
                except BaseException:
                    if launch.cancel_before_commit():
                        self._clear_generation(thread, launch, finished)
                    raise
            else:
                launch = self._disconnect_launch
                finished = self._disconnect_finished
                if launch is None or finished is None:
                    raise RuntimeError(
                        "SIYI disconnect ownership is incomplete"
                    )
        if thread is threading.current_thread():
            return False

        if not launch.committed:
            launch.wait_until_entered(self._remaining_s(deadline_s))
        if not launch.committed and launch.cancel_before_commit():
            with self._lock:
                self._clear_generation(thread, launch, finished)
            return False
        if not finished.wait(self._remaining_s(deadline_s)):
            return False

        with self._lock:
            if (
                self._disconnect_thread is not thread
                or self._disconnect_launch is not launch
                or self._disconnect_finished is not finished
            ):
                return False
            error = self._disconnect_error
            self._disconnect_error = None
            self._disconnect_thread = None
            self._disconnect_launch = None
            self._disconnect_finished = None
            if error is None:
                self._disconnected = True
        if error is not None:
            raise error
        return True

    def _run_disconnect(
        self,
        launch: ThreadLaunchGate,
        finished: threading.Event,
    ) -> None:
        try:
            if not launch.enter():
                return
            result = self._sdk.disconnect()
            if result is False:
                raise TimeoutError("raw SIYI SDK disconnect was incomplete")
        except BaseException as error:
            with self._lock:
                self._disconnect_error = error
        finally:
            finished.set()

    @staticmethod
    def _remaining_s(deadline_s: float) -> float:
        return max(0.0, deadline_s - time.monotonic())

    def _clear_generation(
        self,
        thread: threading.Thread,
        launch: ThreadLaunchGate,
        finished: threading.Event,
    ) -> None:
        if (
            self._disconnect_thread is thread
            and self._disconnect_launch is launch
            and self._disconnect_finished is finished
        ):
            self._disconnect_thread = None
            self._disconnect_launch = None
            self._disconnect_finished = None
            self._disconnect_error = None


__all__ = ["BoundedSiyiSdk", "SDK_DISCONNECT_TIMEOUT_S"]
