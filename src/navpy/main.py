"""NavPy command-line entry point."""

from navpy.app.navpy_argparse import make_parser
from navpy.app.runtime_composition import compose_runtime


def main(args) -> int:
    return compose_runtime(args).run()


if __name__ == "__main__":
    raise SystemExit(
        main(make_parser(description="NavPy Control System").parse_args())
    )


__all__ = ["main"]
