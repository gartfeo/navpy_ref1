from dataclasses import dataclass

from navpy.core.kernel.config.detector_backend import (
    DEFAULT_DETECTOR_BACKEND,
    DETECTOR_BACKENDS,
)


@dataclass
class VisionArgs:
    """Minimal CLI args for vision configuration.

    All camera, gimbal, and detector settings come from vision profiles.
    These args control profile selection and detector type only.
    """

    profile_name: str = ""        # Vision profile (empty = default from JSON)
    detector_type: str = "sim"    # "sim" | "real"
    detector_backend: str = DEFAULT_DETECTOR_BACKEND  # real only: "charuco" | "yolo"
    model_path: str = ""          # YOLO model path (real + yolo backend only)
    debug_show: bool = False      # Show debug window
    # HITL runner only (tools/tuning/approach.py): the run record and a new integration log.
    integration_timing: str = ""
    integration_log: str = ""
    # HITL runner only (step 26): the run's contact link file (tools/hitl/contact_service.py link).
    hitl_contact: str = ""
    siyi_ip: str = ""             # HITL runner only: the SIYI gimbal's address over the profile's siyi_ip
    # HITL commissioning only: a capture wrapper a launcher sets on the parsed args (tools/hitl/media_fault_injector.py);
    # no command-line option sets it.
    capture_wrapper: object = None

    @classmethod
    def from_args(cls, args) -> "VisionArgs":
        """Create VisionArgs from parsed CLI args."""
        return cls(
            profile_name=getattr(args, "vision_profile", ""),
            detector_type=getattr(args, "detector_type", "sim"),
            detector_backend=getattr(
                args, "detector_backend", DEFAULT_DETECTOR_BACKEND
            ),
            model_path=getattr(args, "detector_model_path", ""),
            debug_show=getattr(args, "detector_debug_show", False),
            integration_timing=getattr(args, "integration_timing", ""),
            integration_log=getattr(args, "integration_log", ""),
            hitl_contact=getattr(args, "hitl_contact", ""),
            siyi_ip=getattr(args, "siyi_ip", ""),
            capture_wrapper=getattr(args, "capture_wrapper", None),
        )

    @staticmethod
    def add_args(parser):
        """Add vision arguments to parser."""
        vision_group = parser.add_argument_group("Vision Arguments")
        vision_group.add_argument(
            "--vision-profile",
            type=str,
            default="",
            help="Vision profile name from vision_profiles.json (empty = use default)",
        )
        vision_group.add_argument(
            "--detector-type",
            type=str,
            choices=["sim", "real"],
            default="sim",
            help="Detector type: sim (simulation) or real (camera). Default: sim",
        )
        vision_group.add_argument(
            "--detector-backend",
            type=str,
            choices=list(DETECTOR_BACKENDS),
            default=DEFAULT_DETECTOR_BACKEND,
            help=(
                "Real detector inference backend: charuco (ChArUco dock "
                "reference board, board geometry from the vision profile) or "
                "yolo (requires --detector-model-path). "
                f"Default: {DEFAULT_DETECTOR_BACKEND}"
            ),
        )
        vision_group.add_argument(
            "--detector-model-path",
            type=str,
            default="",
            help=(
                "Path to the YOLO model, relative to the repo root. Required "
                "with --detector-backend=yolo; unused by charuco. No default."
            ),
        )
        vision_group.add_argument(
            "--detector-debug-show",
            action="store_true",
            default=False,
            help="Show debug window for detector",
        )
        vision_group.add_argument(
            "--integration-timing",
            type=str,
            default="",
            metavar="RUN_RECORD",
            help=(
                "HITL runner only: the run record whose integration_timing "
                "section times encoded video (real detector). Needs "
                "--integration-log."
            ),
        )
        vision_group.add_argument(
            "--integration-log",
            type=str,
            default="",
            metavar="FILE",
            help=(
                "HITL runner only: a new JSONL file for the integration "
                "timing log. Needs --integration-timing."
            ),
        )
        vision_group.add_argument(
            "--hitl-contact",
            type=str,
            default="",
            metavar="LINK_FILE",
            help=(
                "HITL runner only: the run's contact link file; the contact "
                "service's authenticated DOCKED events end the episode. Needs "
                "--integration-timing and a profile with hitl_contact."
            ),
        )
        vision_group.add_argument(
            "--siyi-ip",
            type=str,
            default="",
            metavar="ADDRESS",
            help=(
                "HITL runner only: the SIYI gimbal's address, over the vision "
                "profile's siyi_ip (the ZR10 emulator on the Windows host)."
            ),
        )
