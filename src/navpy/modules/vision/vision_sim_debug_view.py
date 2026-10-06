"""Stacked simulator panel assembly and UI dispatch."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from navpy.modules.vision.vision_debug_panel import SimPanelRenderer
from navpy.modules.vision.vision_debug_window import VisionDebugWindow
from navpy.modules.vision.vision_ui_ports import (
    DetectorUiStepPort,
    SimDebugSource,
    SimulatorDebugViewPort,
)


class StackedSimulatorView:
    PANEL_WIDTH = 400
    PANEL_HEIGHT = 200

    def __init__(
        self,
        sources: Sequence[SimDebugSource],
        panel: SimPanelRenderer,
        window: VisionDebugWindow,
    ) -> None:
        self._sources = tuple(sources)
        self._panel = panel
        self._window = window

    def ui_step(self) -> bool:
        panels = []
        for source in self._sources:
            snapshot = source.read_snapshot()
            panels.append(
                (
                    float(snapshot.gimbal.att.pitch),
                    snapshot,
                )
            )
        panels.sort(key=lambda item: -item[0])
        if not panels:
            return True
        image = np.zeros(
            (self.PANEL_HEIGHT * len(panels), self.PANEL_WIDTH, 3),
            dtype=np.uint8,
        )
        for index, (pitch, snapshot) in enumerate(panels):
            self._panel.draw(
                image,
                index * self.PANEL_HEIGHT,
                self.PANEL_WIDTH,
                self.PANEL_HEIGHT,
                snapshot,
                pitch,
            )
        return self._window.show(image)

    def close(self) -> None:
        self._window.close()


class VisionUiDispatcher:
    """Dispatch only to explicitly bound real or simulator UI owners."""

    def __init__(
        self,
        real_detectors: Sequence[DetectorUiStepPort],
        simulator_view: Optional[SimulatorDebugViewPort],
    ) -> None:
        self._real_detectors = tuple(real_detectors)
        self._simulator_view = simulator_view

    def ui_step(self) -> bool:
        for detector in self._real_detectors:
            if not detector.ui_step():
                return False
        if self._simulator_view is not None:
            return self._simulator_view.ui_step()
        return True


__all__ = ["StackedSimulatorView", "VisionUiDispatcher"]
