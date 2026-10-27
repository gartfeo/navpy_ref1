"""NavPy command-line entry point."""

from navpy.app.navpy_argparse import make_parser
from navpy.app.runtime_composition import compose_runtime
from navpy.core.kernel.windows_process_qos import raise_process_qos


def main(args) -> int:
    raise_process_qos()
    from navpy.core.kernel.logging import reader_trace  # TEMP diag
    reader_trace.enable()
    return compose_runtime(args).run()


if __name__ == "__main__":
    raise SystemExit(
        main(make_parser(description="NavPy Control System").parse_args())
    )


__all__ = ["main"]
