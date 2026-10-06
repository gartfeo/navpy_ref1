from dataclasses import dataclass


@dataclass
class VisionArgs:
    """Minimal CLI args for vision configuration.

    All camera, gimbal, and detector settings come from vision profiles.
    These args control profile selection and detector type only.
    """

    profile_name: str = ""        # Vision profile (empty = default from JSON)
    detector_type: str = "sim"    # "sim" | "real"
    model_path: str = ""          # YOLO model path (real only)
    debug_show: bool = False      # Show debug window

    @classmethod
    def from_args(cls, args) -> "VisionArgs":
        """Create VisionArgs from parsed CLI args."""
        return cls(
            profile_name=getattr(args, "vision_profile", ""),
            detector_type=getattr(args, "detector_type", "sim"),
            model_path=getattr(args, "detector_model_path", ""),
            debug_show=getattr(args, "detector_debug_show", False),
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
            help="Detector type: sim (simulation) or real (YOLO). Default: sim",
        )
        vision_group.add_argument(
            "--detector-model-path",
            type=str,
            default=".models/yolov8n-face-lindevs.pt",
            help="Path to YOLO model (real detector only). Default: .models/yolov8n-face-lindevs.pt",
        )
        vision_group.add_argument(
            "--detector-debug-show",
            action="store_true",
            default=False,
            help="Show debug window for detector",
        )
