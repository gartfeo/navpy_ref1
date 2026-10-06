# NavPy

## UAV Swarm Scope

NavPy is a UAV simulation and navigation framework with camera control, detection,
logging, and network messaging.

NavPy develops autonomous navigation and fleet-coordination technology for
cooperative UAV swarms. The swarm core is the product; mission functions are
plug-and-play modules. Example modules are time-critical payload delivery to
authorized recipients and fire detection and suppression (e.g. forest fire
spots). The project is non-weaponized and is not
intended for weapons, explosives, harmful payload delivery, or hostile targeting.

The intended moving-recipient delivery use case is a fixed-wing UAV transporting
a package toward an authorized vehicle traveling through difficult terrain.
The system is intended to estimate the cooperative recipient's motion and plan
a safe rendezvous subject to aircraft, terrain, airspace, energy, communications,
and operational constraints. Rendezvous means an approved delivery configuration
at an appropriate place and time, not physical interception of the vehicle.

The current MVP is a three-UAV delivery/docking simulator demo, with attachable
delivery docks on stationary or moving platforms. It does not yet establish an
end-to-end safe moving-recipient handover, trained dock recognition, or physical
capture performance. See `AGENTS.md` for the project purpose and terminology.

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

Entry points are provided through `main.py` and `main_gui.py`. Run either script after installing the package to start the simulation environment:

```bash
python main.py [options]
```

## MAVLink Messages

NavPy now communicates using custom MAVLink messages defined in `src/navpy/comm/navpy.xml`.
The generated Python classes in `navpy_mavlink.py` allow round‑trip conversion
between dataclass messages and MAVLink packets. See `tests/comm/test_mavlink_conversion.py`
for examples.

Command line arguments for cameras, navigation and networking are defined under the `args/` package.

## Development

Tests are located under the `tests/` directory and can be executed with:

```bash
pip install pytest
```

```bash
pytest tests
```

The repository targets Python 3.8+ and follows standard PEP 8 style.
