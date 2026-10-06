#!/usr/bin/env python3
"""
Check navlink message CRCs match across pymavlink, ArduPilot SITL headers, and
mavlink-router headers.

Unlike the older check_navlink_crcs.py this drives the message id/name list from
navlink.xml (the source of truth) rather than a hardcoded dict, so a freshly
added message is covered automatically. Run inside WSL with the WSL pymavlink so
the CRCs compared are the ones the running swarm will use.

Usage:
    python3 navlink_crc_check.py \
        --xml   ~/ardupilot/modules/mavlink/message_definitions/v1.0/navlink.xml \
        --ardupilot ~/ardupilot \
        --router    ~/mavlink-router \
        [--require-id 25110]

Exit codes:
    0 - all CRCs match (and --require-id, if given, is present everywhere)
    1 - a CRC mismatch / missing message was found
"""
import argparse
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET


TYPE_SIZES = {
    "uint8_t": 1, "int8_t": 1, "char": 1,
    "uint16_t": 2, "int16_t": 2,
    "uint32_t": 4, "int32_t": 4, "float": 4,
    "uint64_t": 8, "int64_t": 8, "double": 8,
}


def parse_navlink_ids(xml_path):
    """Return {msg_id: name} for every message defined in navlink.xml."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    ids = {}
    for msg in root.iter("message"):
        ids[int(msg.attrib["id"])] = msg.attrib["name"]
    return ids


def get_pymavlink_crcs(names_by_id):
    from pymavlink.dialects.v20 import ardupilotmega as mav

    crcs = {}
    for msg_id, name in names_by_id.items():
        class_name = f"MAVLink_{name.lower()}_message"
        if not hasattr(mav, class_name):
            print(f"    WARNING: {name} not found in pymavlink")
            continue
        msg_class = getattr(mav, class_name)
        msg_len = 0
        if getattr(msg_class, "native_format", None):
            try:
                msg_len = struct.calcsize("<" + msg_class.native_format.decode().replace("Z", "s"))
            except Exception:
                msg_len = 0
        if msg_len == 0:
            for ft in msg_class.fieldtypes:
                if "[" in ft:
                    base, count = ft.rstrip("]").split("[")
                    msg_len += TYPE_SIZES.get(base, 4) * int(count)
                else:
                    msg_len += TYPE_SIZES.get(ft, 4)
        crcs[msg_id] = {"name": name, "crc": msg_class.crc_extra, "len": msg_len}
    return crcs


def parse_c_header_crcs(header_path, valid_ids):
    if not os.path.exists(header_path):
        return None
    with open(header_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()
    if not re.search(r"#define MAVLINK_MESSAGE_CRCS\s*\{\{", content):
        return None
    crcs = {}
    for entry in re.findall(r"\{(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*\d+,\s*\d+,\s*\d+\}", content):
        msg_id = int(entry[0])
        if msg_id in valid_ids:
            crcs[msg_id] = {
                "name": valid_ids[msg_id],
                "crc": int(entry[1]),
                "len": int(entry[3]),
            }
    return crcs


def compare(name1, c1, name2, c2):
    errors = []
    if c1 is None or c2 is None:
        return errors
    for msg_id in sorted(set(c1) | set(c2)):
        if msg_id not in c1:
            errors.append(f"{c2[msg_id]['name']} (ID {msg_id}): missing in {name1}")
        elif msg_id not in c2:
            errors.append(f"{c1[msg_id]['name']} (ID {msg_id}): missing in {name2}")
        elif c1[msg_id]["crc"] != c2[msg_id]["crc"]:
            errors.append(
                f"{c1[msg_id]['name']} (ID {msg_id}): CRC mismatch "
                f"{name1}={c1[msg_id]['crc']} vs {name2}={c2[msg_id]['crc']}"
            )
    return errors


def main():
    ap = argparse.ArgumentParser(description="navlink CRC consistency check (xml-driven)")
    ap.add_argument("--xml", required=True, help="navlink.xml source of truth")
    ap.add_argument("--ardupilot", default=os.path.expanduser("~/ardupilot"))
    ap.add_argument("--router", default=os.path.expanduser("~/mavlink-router"))
    ap.add_argument("--require-id", type=int, default=None,
                    help="Fail unless this message id is present in all sources")
    args = ap.parse_args()

    print("=" * 60)
    print("NAVLINK CRC CONSISTENCY CHECK (xml-driven)")
    print("=" * 60)

    names_by_id = parse_navlink_ids(args.xml)
    print(f"navlink.xml defines {len(names_by_id)} messages")

    sources = {}
    print("\n[1] pymavlink...")
    sources["pymavlink"] = get_pymavlink_crcs(names_by_id)
    print(f"    {len(sources['pymavlink'])} messages")

    ardu_hdr = os.path.join(
        args.ardupilot,
        "build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0/ardupilotmega/ardupilotmega.h",
    )
    print(f"\n[2] ArduPilot header: {ardu_hdr}")
    sources["ardupilot"] = parse_c_header_crcs(ardu_hdr, names_by_id)
    print(f"    {len(sources['ardupilot']) if sources['ardupilot'] else 'MISSING (run ./waf plane)'}")

    router_hdr = os.path.join(args.router, "modules/mavlink_c_library_v2/ardupilotmega/ardupilotmega.h")
    print(f"\n[3] mavlink-router header: {router_hdr}")
    sources["router"] = parse_c_header_crcs(router_hdr, names_by_id)
    print(f"    {len(sources['router']) if sources['router'] else 'MISSING'}")

    print("\n" + "=" * 60)
    all_errors = []
    # A missing/unparseable source is a FAILURE, not a skip - otherwise the gate
    # can report PASSED when a header was never built or could not be read.
    for src_name in ("pymavlink", "ardupilot", "router"):
        if not sources.get(src_name):
            all_errors.append(f"{src_name}: source missing or empty (no CRCs parsed)")
            print(f"{src_name}: MISSING/EMPTY")
    for a, b in (("pymavlink", "ardupilot"), ("pymavlink", "router"), ("ardupilot", "router")):
        # compare() tolerates None and yields no rows for it; the missing-source
        # errors above already fail the gate. Compare non-None pairs for detail.
        if sources.get(a) is not None and sources.get(b) is not None:
            errs = compare(a, sources[a], b, sources[b])
            if errs:
                print(f"{a} vs {b}: {len(errs)} error(s)")
                for e in errs:
                    print(f"  - {e}")
                all_errors.extend(errs)
            else:
                print(f"{a} vs {b}: OK")

    if args.require_id is not None:
        rid = args.require_id
        for src_name in ("pymavlink", "ardupilot", "router"):
            src = sources.get(src_name)
            if not src or rid not in src:
                all_errors.append(f"required id {rid} missing in {src_name}")
                print(f"required id {rid}: MISSING in {src_name}")
        if not all_errors:
            print(f"required id {rid}: present in all sources")

    print("=" * 60)
    if all_errors:
        print(f"FAILED: {len(all_errors)} error(s)")
        return 1
    print("PASSED: All CRCs match!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
