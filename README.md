# NavPy

NavPy is a cooperative UAV swarm framework: autonomous navigation, fleet
coordination, vision-based final approach, camera control, detection, logging
and network messaging for multi-UAV missions. The swarm core is the product and
is mission-agnostic; mission functions are plug-and-play modules, so the same
swarm serves factory and industrial survey/inspection, agricultural spraying,
border security surveillance, forest fire detection and suppression with
special equipment, and time-critical medicine or payload delivery to authorized
recipients. NavPy is for civilian use only.

See `AGENTS.md` for the project purpose and glossary (POI, dock, final
approach, default delivery hub, swarm, fleet).

## Mission modules

### Delivery (current MVP demo)

The current MVP is a three-UAV delivery/docking simulator demo, with docks on
stationary or moving platforms.

The intended moving-recipient use case is a fixed-wing UAV transporting a
package toward an authorized vehicle traveling through difficult terrain. The
system is intended to estimate the cooperative recipient's motion and plan a
safe rendezvous subject to aircraft, terrain, airspace, energy, communications,
and operational constraints. Rendezvous means a cooperative capture at an
agreed place and time: the UAV flies into capture equipment that the recipient
deploys and controls, such as the dock net on the vehicle.

The demo does not yet establish an end-to-end safe moving-recipient handover,
trained dock recognition, or physical capture performance.

## Installation

The project can be installed in editable mode using `pip`:

```bash
pip install -e .
```

Runtime dependencies are declared in `pyproject.toml` and are installed with the
command above. There is no root `requirements.txt`.

GCS backend work and running `pytest tests/gcs` need two more Python installs —
the backend's own runtime dependencies, and the `test` extra:

```bash
pip install -r src/gcs/requirements.txt
pip install -e ".[test]"
```

Without the `test` extra, the TestClient-based GCS test modules fail during
collection, before any of their tests run, because starlette imports `httpx2` at
import time.

Frontend work additionally needs `npm install` in `src/gcs/frontend`. The Vitest
bridge in `tests/gcs` skips itself when those packages are absent.

## Usage

After installing, run NavPy with the `navpy` console script or as a module:

```bash
python -m navpy.main [options]      # or: python -m navpy.main_gui [options]
```

To run the GCS app with SITL, use the isolated launcher described in
`AGENTS.md` ("Running the GCS stack").

## Detector and arguments

Command line arguments for cameras, navigation, vision and networking are
defined under `src/navpy/args/`. `--detector-type` selects `sim` (default) or
`real`; the real detector defaults to a ChArUco board detector
(`--detector-backend charuco`, board geometry from the vision profile). The
YOLO face model is a bench-only opt-in: `--detector-backend yolo
--detector-model-path <path>`.

## Swarm messages

Swarm messages are dataclasses under `src/navpy/modules/comm/messages/` that
convert to and from MAVLink packets of the project's pymavlink fork (declared
in `pyproject.toml`). See `tests/modules/comm/test_mavlink_conversion.py` for
round-trip examples.

## Development

Tests live under `tests/`:

```bash
PYTHONPATH="$PWD/src" python -m pytest tests
```

The repository targets Python 3.9+ (`requires-python` in `pyproject.toml`) and
follows standard PEP 8 style.
