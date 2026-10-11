"""Write step1_exceptions.tsv: layer ratchet rows (from check_rules --list on the effective step-1 graph)
plus pure-vision rows (frontier edges from vision_reach.py), each with a fix step and a reason.

Usage: python gen_exceptions.py <eff_dir> <old_exceptions.tsv> <out.tsv>
"""
import csv, sys

eff, old_path, out_path = sys.argv[1:4]
man = {r["src"]: r for r in csv.DictReader(open(f"{eff}/eff_manifest_step1.tsv", encoding="utf-8"), delimiter="\t")}
old = {(r["file"], r["imports"]): r for r in csv.DictReader(open(old_path, encoding="utf-8"), delimiter="\t")}
FIX_OVERRIDE = {
    ("core/flow/mission_catalog.py", "plugins/vehicle/ardupilot/mission_metadata.py"):
        ("C3 vehicle port + R16",
         "catalog reads our mission metadata through the ArduPilot reader; C3 gives raw mission items through the "
         "vehicle port, R16 moves catalog and DDH reader to missions/delivery"),
}


def dest(k):
    return man[k]["step1_dest"]


rows = []
for r in csv.DictReader(open(f"{eff}/v_step1.tsv", encoding="utf-8"), delimiter="\t"):
    f, t = dest(r["src"]), dest(r["imports"])
    o = old.get((f, t))
    fix, why = FIX_OVERRIDE.get((f, t), (o["fix_step"], o["why"]) if o else ("?", "?"))
    rows.append(["layer", f, t, r["from_layer"], r["to_layer"], r["where"],
                 "yes" if r["type_checking"] == "True" else "no", fix, why, r["src"].split("#")[0]])

GIMBAL = ("C3 gimbal split",
          "geo pointing of the gimbal is composed with the detector and the gimbal owner; after the split, search "
          "and observe skills send geo requests to core/command and the vision path keeps only readback and tracking")
LEGACY = ("R9 geo-approach skill",
          "legacy geo branch inside the flow decision (PID/PN pass tracking); runs only when no final approach runs")
VISION = {
    ("core/approach/vision_nav/diagnostics.py", "core/perception/geo/geo_ref_calc.py"):
        ("allowed (log only)", "SNAP log row distance, computed after the command is formed; never feeds it"),
    ("core/approach/navigation_lifecycle.py", "core/approach/geo/legacy_destination_resolver.py"):
        ("R8 law registry", "lifecycle resets LegacyNavigationState; R8 puts the legacy reset behind the law's hook"),
    ("core/flow/mission_navigation.py", "core/perception/geo/geo_ref_calc.py"): LEGACY,
    ("core/flow/mission_navigation.py", "core/skills/geo_approach/pass_tracker.py"): LEGACY,
    ("core/flow/navigation_task_reset.py", "core/skills/geo_approach/pass_tracker.py"):
        ("R9 geo-approach skill", "task reset clears the legacy pass tracker; the skill owns its reset after R9"),
    ("core/skills/confirm/track_recovery.py", "core/perception/geo/geo_ref_calc.py"):
        ("R10 runner", "NAV decision calls confirm-skill internals (track-loss geo hold); only the runner picks edges "
                       "after R10"),
    ("plugins/vehicle/ardupilot/position_service.py", "core/perception/geo/zc_util_instance.py"):
        ("C1 registry", "vision files reach the whole vehicle facade (vehicle_public_ports), whose position service "
                        "reads the terrain singleton; C1 has app inject the lookup (same edge as the layer row)"),
}
for line in open(f"{eff}/vision_frontier.txt", encoding="utf-8"):
    line = line.strip()
    if "->" not in line:
        continue
    left, right = [x.strip() for x in line.split("->", 1)]
    right = right.split()[0]
    f, ln = left.split(":")
    where = "function" if "(fn)" in ln else "top"
    fix, why = VISION.get((f, right), GIMBAL)
    lf = "/".join(f.split("/")[:2])
    lt = "/".join(right.split("/")[:2])
    src = next(k for k in man if dest(k) == f)
    rows.append(["vision", f, right, lf, lt, where, "no", fix, why, src.split("#")[0]])

with open(out_path, "w", encoding="utf-8", newline="") as fh:
    w = csv.writer(fh, delimiter="\t", lineterminator="\n")
    w.writerow(["guard", "file", "imports", "from_layer", "to_layer", "where", "type_checking", "fix_step", "why",
                "old_src"])
    w.writerows(sorted(rows))
print("layer", sum(r[0] == "layer" for r in rows), "vision", sum(r[0] == "vision" for r in rows),
      "unknown fix", sum(r[7] == "?" for r in rows))
