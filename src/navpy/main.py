"""NavPy command-line entry point."""

from navpy.app.navpy_argparse import make_parser
from navpy.app.runtime_composition import compose_runtime
from navpy.core.kernel.logging.pose_cadence_gc import install_gc_record
from navpy.core.kernel.windows_process_qos import raise_process_qos


def main(args) -> int:
    raise_process_qos()
    # Wall entry only: lockstep composes the runtime without this module.
    install_gc_record(args.source_system)
    return compose_runtime(args).run()


if __name__ == "__main__":
    raise SystemExit(
        main(make_parser(description="NavPy Control System").parse_args())
    )


__all__ = ["main"]
