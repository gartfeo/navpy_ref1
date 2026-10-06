"""Declare every flight before launch; stop and preserve the first failed attempt."""

import argparse
from datetime import datetime
import itertools
import json
from pathlib import Path

from eval_lockstep_fleet import ROOT, run
from simtime_step_launch import require_base_interpreter


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    args = parser.parse_args()
    require_base_interpreter()
    root = ROOT / ".sitl-runs" / f"lockstep-matrix-{datetime.now():%Y%m%d-%H%M%S}"
    root.mkdir()
    cells = [(1, 1., False)] + [(3, speed, delay) for repeat, speed, delay
                                 in itertools.product(range(3), (10., 1.), (False, True))]
    ledger = dict(version=1, policy="one attempt per declared cell; abort on first rejection",
                  repetitions=3, attempts=[dict(directory=str(root / f"run-{i:02}"),
                  instances=count, speedup=speed, delayed=delay, status="pending")
                  for i, (count, speed, delay) in enumerate(cells)])
    path = root / "attempts.json"
    def save() -> None:
        path.write_text(json.dumps(ledger, indent=2))
    save()
    print(f"LEDGER {path}", flush=True)
    for attempt in ledger['attempts']:
        attempt['status'] = 'running'
        save()
        options = argparse.Namespace(firmware_root=args.firmware_root,
            peer_python=args.peer_python, **{k: attempt[k] for k in ('instances', 'speedup', 'delayed')})
        try:
            result = run(options, Path(attempt['directory']))
            attempt.update(status='captured', run_id=result['run_id'])
        except BaseException as error:
            attempt.update(status='rejected', error=str(error))
            save()
            raise
        save()
        print(json.dumps(attempt), flush=True)


if __name__ == '__main__':
    main()
