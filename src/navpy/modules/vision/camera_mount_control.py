"""Camera-mount lifecycle and transactional zoom commands."""

from __future__ import annotations

import threading

from navpy.modules.vision.camera_mount_ports import (
    CameraLifecyclePort,
    CameraZoomPort,
    ControlZoomReadbackPort,
    GimbalLifecyclePort,
    GimbalZoomCommandPort,
)


class MountZoomControl:
    """Keep physical zoom and camera intrinsics transactionally aligned."""

    def __init__(
        self,
        camera: CameraZoomPort,
        gimbal: GimbalZoomCommandPort,
        readback: ControlZoomReadbackPort,
        lock: threading.RLock,
    ) -> None:
        self._camera = camera
        self._gimbal = gimbal
        self._readback = readback
        self._lock = lock

    def sync_from_hardware(self) -> bool:
        with self._lock:
            command = self._readback.zoom_key_from_hardware()
            if command is None or self._readback.camera_zoom_key() == command:
                return False
            return self._camera.apply_zoom(command).accepted

    def command_hardware(self, zoom: str | float) -> bool:
        return self._gimbal.apply_zoom(str(zoom)).committed

    def set_zoom(self, zoom: str | float) -> bool:
        with self._lock:
            return self._set_locked(str(zoom))

    def _set_locked(self, command: str) -> bool:
        current = self._readback.current_zoom()
        camera_write = self._camera.apply_zoom(command)
        if camera_write.supported and not camera_write.accepted:
            return False
        prepared_camera = camera_write.accepted

        hardware_write = self._gimbal.apply_zoom(command)
        if hardware_write.supported and hardware_write.result is False:
            if prepared_camera and current is not None and current != command:
                self._camera.apply_zoom(current)
            return False
        return prepared_camera or hardware_write.committed


class MountLifecycle:
    """Preserve the historical camera/gimbal lifecycle ordering."""

    def __init__(
        self,
        camera: CameraLifecyclePort,
        gimbal: GimbalLifecyclePort,
        lock: threading.RLock,
    ) -> None:
        self._camera = camera
        self._gimbal = gimbal
        self._lock = lock

    def start(self) -> None:
        self._gimbal.start()

    def stop(self) -> bool:
        return self._gimbal.stop()

    def refresh(self) -> None:
        with self._lock:
            self._camera.refresh()
        self._gimbal.refresh()

    def raise_if_failed(self) -> None:
        self._camera.raise_if_failed()
        self._gimbal.raise_if_failed()


__all__ = ["MountLifecycle", "MountZoomControl"]
