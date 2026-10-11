"""Vision module - Detection, cameras, gimbals, and POI tracking.

This module owns ALL vision-related functionality:
- VisionController - Self-contained orchestrator for all vision components
- CameraMount - Encapsulates camera + gimbal
- Detector interfaces and implementations
- Camera interfaces and simulation
- Gimbal interfaces and simulation
- Visual-reference models and simulator POI providers

A detector observation or stable track preserves POI association only;
approach accuracy does not establish a mission outcome (e.g. docking, cargo receipt, coverage).
"""
