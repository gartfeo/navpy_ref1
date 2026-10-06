"""NavPy command-line entry point."""

from navpy.args.conn_args import ConnArgs
from navpy.args.navpy_argparse import make_parser
from navpy.runtime_composition import compose_runtime


def main(args) -> int:
    return compose_runtime(args).run()


if __name__ == "__main__":
    raise SystemExit(
        main(make_parser(description="NavPy Control System").parse_args())
    )


__all__ = ["ConnArgs", "main"]
