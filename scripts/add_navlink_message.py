#!/usr/bin/env python3
r"""
add_navlink_message - one-shot helper to add a REAL navlink MAVLink message.

Run this on Windows. It orchestrates the whole cross-OS chain: the Windows-side
navpy edits happen in-process, and every ArduPilot/mavlink-router/pymavlink build
step is driven inside WSL via ``wsl.exe``.

Why a full rebuild is unavoidable: arduplane FORWARDS companion<->GCS traffic and
DROPS message ids it was not compiled with, so ``./waf plane`` must run. This is
NOT a "skip the firmware rebuild" tool - it makes the required chain one command.

Given a message spec (JSON), it will:
  1. scaffold the navpy wrapper class + MsgType enum + ttl default + a roundtrip test
  2. (WSL) insert the <message> block into navlink.xml (source of truth)
  3. (WSL) reinstall pymavlink from the local source (regenerates the dialect)
  4. (WSL) copy navlink.xml -> c_library_v2 and regen the FULL ardupilotmega C headers
  5. (WSL) rebuild mavlink-router (ninja) and ArduPilot SITL (./waf plane)
  6. push navlink.xml to the fork branch, then reinstall the Windows .venv pymavlink
  7. (WSL) CRC gate: pymavlink vs ArduPilot header vs router header must all match
  8. (WSL) routing proof: launch a small SITL swarm and send the message end-to-end

Usage:
    python scripts/add_navlink_message.py --message task_confirm_ack
    python scripts/add_navlink_message.py --spec scripts/navlink_messages/foo.json --dry-run
    python scripts/add_navlink_message.py --message foo --no-push --no-route-test

Flags:
    --message NAME     resolve scripts/navlink_messages/<name>.json
    --spec PATH        explicit spec path (overrides --message)
    --dry-run          print the full plan, change nothing
    --skip-waf         skip ./waf plane (FASTER but the message will be dropped by
                       arduplane in a live swarm - only for iterating on wrappers)
    --no-push          skip fork push + Windows git-reinstall (Windows .venv will
                       NOT get the message)
    --no-route-test    skip the live swarm routing proof
    --instances N      swarm size for the routing proof (default 2)
    --venv PATH        Windows venv dir whose pymavlink to reinstall
                       (default: <repo>/.venv)
"""
import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
META_FIELDS = ("boot_id", "msg_seq", "time_ms", "ttl_ms")

# Strict patterns: spec values are interpolated into bash/python command text, so
# identifier-like fields must be validated (never trust the JSON) to prevent shell
# injection and malformed builds.
MSG_NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")       # MAVLink message / enum name
FIELD_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")     # mavlink field name
CLASS_NAME_RE = re.compile(r"^[A-Z][A-Za-z0-9_]*$")  # python class name
BASE_TYPES = {
    "uint8_t", "int8_t", "char", "uint16_t", "int16_t",
    "uint32_t", "int32_t", "float", "uint64_t", "int64_t", "double",
}
FIELD_TYPE_RE = re.compile(r"^(" + "|".join(sorted(BASE_TYPES)) + r")(\[\d+\])?$")
FORK_GIT = "https://github.com/gartfeo/mavlink.git"
FORK_BRANCH = "ArduPilot-4.6/navlink"

# WSL locations (evaluated inside `bash -lc`, so $HOME expands).
WSL_ARDU = "$HOME/ardupilot"
WSL_ROUTER = "$HOME/mavlink-router"
WSL_MAVLINK = "$HOME/ardupilot/modules/mavlink"
WSL_PYMAVLINK = "$HOME/ardupilot/modules/mavlink/pymavlink"
WSL_NAVLINK_XML = "$HOME/ardupilot/modules/mavlink/message_definitions/v1.0/navlink.xml"
WSL_CLIB = "$HOME/mavlink-router/modules/mavlink_c_library_v2"
WSL_MAVGEN = "$HOME/.local/bin/mavgen.py"
WSL_RUN_SWARM = "$HOME/ardupilot/Tools/autotest/run_swarm.sh"


# --------------------------------------------------------------------------- #
# small utilities
# --------------------------------------------------------------------------- #
def log(msg):
    print(f"[add_navlink] {msg}", flush=True)


def win_to_wsl(path):
    """C:\\repos\\x -> /mnt/c/repos/x"""
    path = os.path.abspath(path)
    drive, rest = os.path.splitdrive(path)
    return "/mnt/" + drive[0].lower() + rest.replace("\\", "/")


def run(cmd_list, dry_run, cwd=None):
    log("RUN " + " ".join(cmd_list))
    if dry_run:
        return 0
    return subprocess.run(cmd_list, cwd=cwd).returncode


def wsl(bash_cmd, dry_run, check=True):
    """Run a bash command inside WSL."""
    log("WSL $ " + bash_cmd)
    if dry_run:
        return 0
    rc = subprocess.run(["wsl.exe", "-e", "bash", "-lc", bash_cmd]).returncode
    if check and rc != 0:
        raise SystemExit(f"WSL step failed (rc={rc}): {bash_cmd}")
    return rc


def load_spec(args):
    if args.spec:
        spec_path = os.path.abspath(args.spec)
    elif args.message:
        spec_path = os.path.join(REPO_ROOT, "scripts", "navlink_messages", f"{args.message.lower()}.json")
    else:
        raise SystemExit("provide --message NAME or --spec PATH")
    if not os.path.exists(spec_path):
        raise SystemExit(f"spec not found: {spec_path}")
    with open(spec_path, "r", encoding="utf-8") as f:
        spec = json.load(f)
    _validate_spec(spec)
    return spec, spec_path


def _validate_spec(spec):
    for key in ("name", "id", "description", "fields", "navpy"):
        if key not in spec:
            raise SystemExit(f"spec missing '{key}'")
    # name / id
    if not MSG_NAME_RE.match(str(spec["name"])):
        raise SystemExit(f"invalid message name '{spec['name']}' (must match {MSG_NAME_RE.pattern})")
    try:
        msg_id = int(spec["id"])
    except (TypeError, ValueError):
        raise SystemExit(f"id must be an integer, got {spec['id']!r}")
    if not (1 <= msg_id <= 0xFFFFFF):
        raise SystemExit(f"id {msg_id} out of range (navlink convention: 25000-25999)")
    # fields
    field_names = [f["name"] for f in spec["fields"]]
    if list(field_names[:4]) != list(META_FIELDS):
        raise SystemExit(
            f"first four fields must be {META_FIELDS} (dedup/TTL meta), got {field_names[:4]}"
        )
    seen = set()
    for f in spec["fields"]:
        if not FIELD_NAME_RE.match(str(f.get("name", ""))):
            raise SystemExit(f"invalid field name '{f.get('name')}' (must match {FIELD_NAME_RE.pattern})")
        if f["name"] in seen:
            raise SystemExit(f"duplicate field name '{f['name']}'")
        seen.add(f["name"])
        if not FIELD_TYPE_RE.match(str(f.get("type", ""))):
            raise SystemExit(f"invalid field type '{f.get('type')}' for '{f['name']}'")
    # navpy
    nav = spec["navpy"]
    for key in ("class_name", "msg_type_enum", "msg_type_value"):
        if key not in nav:
            raise SystemExit(f"spec.navpy missing '{key}'")
    if not CLASS_NAME_RE.match(str(nav["class_name"])):
        raise SystemExit(f"invalid navpy.class_name '{nav['class_name']}'")
    if not MSG_NAME_RE.match(str(nav["msg_type_enum"])):
        raise SystemExit(f"invalid navpy.msg_type_enum '{nav['msg_type_enum']}'")
    try:
        int(nav["msg_type_value"])
    except (TypeError, ValueError):
        raise SystemExit(f"navpy.msg_type_value must be an integer, got {nav['msg_type_value']!r}")
    rf = nav.get("receiver_field")
    if rf and rf not in field_names:
        raise SystemExit(f"receiver_field '{rf}' is not a declared field")


# --------------------------------------------------------------------------- #
# Windows-side navpy scaffolding
# --------------------------------------------------------------------------- #
def _payload_fields(spec):
    return [f for f in spec["fields"] if f["name"] not in META_FIELDS]


def generate_wrapper(spec):
    name = spec["name"]
    nav = spec["navpy"]
    cls = nav["class_name"]
    enum = nav["msg_type_enum"]
    receiver_field = nav.get("receiver_field")
    payload = _payload_fields(spec)
    attr_fields = [f["name"] for f in payload if f["name"] != receiver_field]

    # constructor params: sender_id, [receiver_id], <attr fields...>, meta
    ctor_params = ["self", "sender_id"]
    if receiver_field:
        ctor_params.append("receiver_id")
    ctor_params += attr_fields
    ctor_params.append("meta=None")
    ctor_sig = ", ".join(ctor_params)

    super_call = (
        "super().__init__(sender_id, receiver_id, meta=meta)"
        if receiver_field
        else "super().__init__(sender_id, meta=meta)"
    )
    store_lines = "\n".join(f"        self.{a} = {a}" for a in attr_fields) or "        pass"

    to_dict_updates = "".join(f'            "{a}": self.{a},\n' for a in attr_fields)

    from_dict_lines = ['            sender_id=data["sid"],']
    if receiver_field:
        from_dict_lines.append('            receiver_id=data.get("rid"),')
    for a in attr_fields:
        from_dict_lines.append(f'            {a}=data["{a}"],')
    from_dict_body = "\n".join(from_dict_lines)

    # to_mavlink positional args, in navlink.xml field order
    mav_args = ["boot_id", "msg_seq", "time_ms", "ttl_ms"]
    for f in payload:
        if f["name"] == receiver_field:
            mav_args.append("self.receiver_id if self.receiver_id is not None else 0")
        else:
            mav_args.append(f"self.{f['name']}")
    mav_args_str = ",\n            ".join(mav_args)

    from_mav_lines = ["            sender_id=mav_msg.get_srcSystem(),"]
    if receiver_field:
        from_mav_lines.append(f"            receiver_id=mav_msg.{receiver_field},")
    for a in attr_fields:
        from_mav_lines.append(f"            {a}=mav_msg.{a},")
    from_mav_lines.append("            meta=meta,")
    from_mav_body = "\n".join(from_mav_lines)

    return f'''"""
{cls} - navpy wrapper for the {name} navlink message.

AUTO-SCAFFOLDED by scripts/add_navlink_message.py. Handles flat (scalar) fields
1:1. If {name} has array fields, extend to_mavlink/from_mavlink by hand.

Import this module where {name} is produced or consumed so @register_msg runs
(navpy has no central message loader; each consumer imports its own message).
"""
from typing import Any, Dict, Optional

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_{name.lower()}_message,
    MAVLINK_MSG_ID_{name.upper()},
)

from navpy.modules.comm.messages.msg_abc import register_msg, MsgABC
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType


def _meta4(meta: Optional[MsgMeta]):
    if meta:
        return meta.boot_id, meta.msg_seq, meta.time_ms, meta.ttl_ms
    return 0, 0, 0, 0


@register_msg()
class {cls}(MsgABC):
    """Wrapper for the {name} navlink message. Includes dedup/TTL metadata."""

    def __init__({ctor_sig}):
        {super_call}
{store_lines}

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data.update({{
{to_dict_updates}        }})
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "{cls}":
        return cls(
{from_dict_body}
        )

    def to_mavlink(self) -> MAVLink_{name.lower()}_message:
        boot_id, msg_seq, time_ms, ttl_ms = _meta4(self.meta)
        return MAVLink_{name.lower()}_message(
            {mav_args_str},
        )

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_{name.lower()}_message) -> "{cls}":
        meta = MsgMeta(
            boot_id=mav_msg.boot_id,
            msg_seq=mav_msg.msg_seq,
            time_ms=mav_msg.time_ms,
            ttl_ms=mav_msg.ttl_ms,
        )
        return cls(
{from_mav_body}
        )

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.{enum}

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_{name.upper()}
'''


def generate_test(spec, module_name):
    name = spec["name"]
    nav = spec["navpy"]
    cls = nav["class_name"]
    receiver_field = nav.get("receiver_field")
    payload = _payload_fields(spec)
    attr_fields = [f["name"] for f in payload if f["name"] != receiver_field]

    ctor_args = ["sender_id=7"]
    if receiver_field:
        ctor_args.append("receiver_id=42")
    for i, a in enumerate(attr_fields):
        ctor_args.append(f"{a}={i + 1}")
    ctor_args_str = ", ".join(ctor_args)

    asserts = []
    if receiver_field:
        asserts.append("    assert rt.receiver_id == 42")
    for i, a in enumerate(attr_fields):
        asserts.append(f"    assert rt.{a} == {i + 1}")
    asserts_str = "\n".join(asserts) or "    pass"

    return f'''"""Roundtrip tests for {cls} ({name}). AUTO-SCAFFOLDED."""
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.{module_name} import {cls}


def _make():
    meta = MsgMeta(boot_id=111, msg_seq=222, time_ms=333, ttl_ms=5000)
    return {cls}({ctor_args_str}, meta=meta)


def test_dict_roundtrip():
    rt = {cls}.from_dict(_make().to_dict())
    assert rt.sender_id == 7
{asserts_str}


def test_mavlink_roundtrip():
    msg = _make()
    rt = {cls}.from_mavlink(msg.to_mavlink())
{asserts_str}
    assert rt.meta.boot_id == 111
    assert rt.meta.ttl_ms == 5000
'''


def scaffold_navpy(spec, dry_run):
    name = spec["name"]
    nav = spec["navpy"]
    module_name = f"{name.lower()}_msg"
    wrapper_path = os.path.join(REPO_ROOT, "src", "navpy", "modules", "comm", "messages", f"{module_name}.py")
    test_path = os.path.join(REPO_ROOT, "tests", "modules", "comm", f"test_{module_name}.py")

    # wrapper
    if os.path.exists(wrapper_path):
        log(f"wrapper exists, skip: {wrapper_path}")
    else:
        log(f"scaffold wrapper: {wrapper_path}")
        if not dry_run:
            with open(wrapper_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(generate_wrapper(spec))

    # test
    if os.path.exists(test_path):
        log(f"test exists, skip: {test_path}")
    else:
        log(f"scaffold test: {test_path}")
        if not dry_run:
            with open(test_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(generate_test(spec, module_name))

    _patch_msgtype(nav["msg_type_enum"], int(nav["msg_type_value"]), dry_run)
    _patch_ttl(nav["msg_type_enum"], int(spec.get("ttl_ms", 5000)), dry_run)


def _patch_msgtype(enum_name, value, dry_run):
    path = os.path.join(REPO_ROOT, "src", "navpy", "modules", "comm", "messages", "types.py")
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    if f"{enum_name} =" in content:
        log(f"MsgType.{enum_name} already present, skip")
        return
    anchor = "    UNKNOWN = 0  # Unrecognized message types"
    if anchor not in content:
        raise SystemExit("could not find MsgType.UNKNOWN anchor in types.py")
    new = content.replace(anchor, f"    {enum_name} = {value}\n{anchor}", 1)
    log(f"add MsgType.{enum_name} = {value}")
    if not dry_run:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(new)


def _patch_ttl(enum_name, ttl_ms, dry_run):
    path = os.path.join(REPO_ROOT, "src", "navpy", "modules", "comm", "messages", "ttl_defaults.py")
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    if f"MsgType.{enum_name}:" in content:
        log(f"ttl for {enum_name} already present, skip")
        return
    anchor = "    # Unknown/fallback"
    if anchor not in content:
        raise SystemExit("could not find '# Unknown/fallback' anchor in ttl_defaults.py")
    new = content.replace(anchor, f"    MsgType.{enum_name}: {ttl_ms},\n\n{anchor}", 1)
    log(f"add ttl default MsgType.{enum_name} = {ttl_ms}")
    if not dry_run:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(new)


# --------------------------------------------------------------------------- #
# WSL build chain
# --------------------------------------------------------------------------- #
def wsl_insert_xml(spec_path, dry_run):
    scripts_wsl = win_to_wsl(os.path.join(REPO_ROOT, "scripts"))
    spec_wsl = win_to_wsl(spec_path)
    wsl(f'python3 "{scripts_wsl}/navlink_xml_insert.py" --xml {WSL_NAVLINK_XML} --spec "{spec_wsl}"', dry_run)


def wsl_reinstall_pymavlink(spec, dry_run):
    wsl(f'set -o pipefail; cd {WSL_PYMAVLINK} && python3 -m pip install --user --force-reinstall --no-deps . 2>&1 | tail -3', dry_run)
    name = spec["name"].upper()
    wsl(
        f'python3 -c "from pymavlink.dialects.v20 import ardupilotmega as m; '
        f'print(\'{name} id =\', m.MAVLINK_MSG_ID_{name})"',
        dry_run,
    )


def wsl_regen_headers(dry_run):
    wsl(f'cp {WSL_NAVLINK_XML} {WSL_CLIB}/message_definitions/navlink.xml', dry_run)
    wsl(
        f'cd {WSL_CLIB} && {WSL_MAVGEN} --lang=C --wire-protocol=2.0 -o . '
        f'message_definitions/ardupilotmega.xml',
        dry_run,
    )


def wsl_rebuild_router(dry_run):
    # /usr/bin/mavlink-routerd is a symlink to build/src/mavlink-routerd, so ninja is enough.
    wsl(f'set -o pipefail; cd {WSL_ROUTER} && ninja -C build 2>&1 | tail -5', dry_run)


def wsl_rebuild_sitl(dry_run):
    wsl(f'set -o pipefail; cd {WSL_ARDU} && ./waf plane 2>&1 | tail -8', dry_run)


def push_and_reinstall_windows(spec, venv_python, dry_run):
    name = spec["name"]  # validated: ^[A-Z][A-Z0-9_]*$
    msg = shlex.quote(f'Add {name} navlink message ({int(spec["id"])})')
    wsl(
        f'cd {WSL_MAVLINK} && git add message_definitions/v1.0/navlink.xml && '
        f'(git diff --cached --quiet && echo "nothing to commit" || git commit -m {msg}) && '
        f'git push origin {FORK_BRANCH}',
        dry_run,
    )
    spec_url = f"pymavlink @ git+{FORK_GIT}@{FORK_BRANCH}#egg=pymavlink&subdirectory=pymavlink"
    rc = run(
        [venv_python, "-m", "pip", "install", "--force-reinstall", "--no-deps", spec_url],
        dry_run,
    )
    if rc:
        raise SystemExit(f"Windows pymavlink reinstall failed (rc={rc})")
    name_u = name.upper()
    if not dry_run:
        rc = subprocess.run(
            [venv_python, "-c",
             f"from pymavlink.dialects.v20 import ardupilotmega as m; "
             f"print('windows {name_u} id =', m.MAVLINK_MSG_ID_{name_u})"]
        ).returncode
        if rc != 0:
            raise SystemExit("Windows pymavlink reinstall did not expose the new message")


def wsl_crc_gate(spec, dry_run):
    scripts_wsl = win_to_wsl(os.path.join(REPO_ROOT, "scripts"))
    wsl(
        f'python3 "{scripts_wsl}/navlink_crc_check.py" --xml {WSL_NAVLINK_XML} '
        f'--ardupilot {WSL_ARDU} --router {WSL_ROUTER} --require-id {spec["id"]}',
        dry_run,
    )


def wsl_route_test(spec, instances, dry_run):
    """Launch a small swarm, send the message, tear down only our own PIDs."""
    name = spec["name"]
    # Sample field values for the send. Address fields go to 0 so mavlink-router
    # BROADCASTS to every endpoint (incl. the second test client). A nonzero
    # target_system would make the router point-route the message to that system's
    # endpoint only, and the client-to-client test harness would see nothing -
    # which looks like a drop but is just addressed delivery.
    broadcast_fields = {"target_system", "target_component"}
    kv = []
    for f in spec["fields"]:
        val = 0 if f["name"] in broadcast_fields else 1
        kv.append(f'{f["name"]}={val}')
    kv_str = " ".join(kv)
    test_py = f"{WSL_MAVLINK}/test_navlink_msg.py"
    # setsid gives the swarm its OWN process group so both readiness counting and
    # teardown target ONLY our PIDs (never broad-kill, and never mistake another
    # session's swarm for ours). An EXIT/INT/TERM trap guarantees teardown even if
    # the shell is interrupted before the inline kill; it preserves the test's rc.
    script = f"""
LOG=/tmp/navlink_swarm_$$.log
setsid {WSL_RUN_SWARM} wsl -n {instances} > "$LOG" 2>&1 &
SWARM_PGID=$!
RC=1
teardown() {{ echo "tearing down swarm (own pgid $SWARM_PGID only)"; kill -TERM -$SWARM_PGID 2>/dev/null || true; sleep 2; kill -KILL -$SWARM_PGID 2>/dev/null || true; }}
trap 'teardown; exit $RC' INT TERM
echo "swarm pgid=$SWARM_PGID log=$LOG"
echo 'waiting for {instances} arduplane instance(s) in our process group to boot...'
READY=0
for i in $(seq 1 60); do
  N=$(pgrep -g $SWARM_PGID -c -x arduplane 2>/dev/null || true)
  if [ "${{N:-0}}" -ge {instances} ]; then echo "arduplane up ($N)"; READY=1; break; fi
  sleep 2
done
if [ "$READY" -ne 1 ]; then
  echo "ERROR: our swarm did not reach {instances} arduplane instance(s); see $LOG"
  teardown
  exit 1
fi
sleep 12   # let heartbeats/router settle
python3 {test_py} {name} {kv_str} --timeout 8
RC=$?
echo "route test rc=$RC"
teardown
exit $RC
"""
    wsl(script, dry_run)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--message", help="resolve scripts/navlink_messages/<name>.json")
    ap.add_argument("--spec", help="explicit spec JSON path")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-waf", action="store_true")
    ap.add_argument("--no-push", action="store_true")
    ap.add_argument("--no-route-test", action="store_true")
    ap.add_argument("--route-only", action="store_true",
                    help="skip build/scaffold/push; only run the live routing proof "
                         "(build must already be done)")
    ap.add_argument("--instances", type=int, default=2)
    ap.add_argument("--venv", default=os.path.join(REPO_ROOT, ".venv"))
    args = ap.parse_args()

    spec, spec_path = load_spec(args)
    venv_python = os.path.join(args.venv, "Scripts", "python.exe")

    log(f"message: {spec['name']} (id {spec['id']})  spec: {spec_path}")
    if args.dry_run:
        log("DRY RUN - no changes will be made")
    if args.skip_waf:
        log("WARNING: --skip-waf: arduplane will DROP this id in a live swarm")
    if args.no_push:
        log("WARNING: --no-push: Windows .venv pymavlink will NOT get the message")

    t0 = time.time()

    if args.route_only:
        log("ROUTE-ONLY: skipping scaffold/build/push, running routing proof only")
        wsl_route_test(spec, args.instances, args.dry_run)
        log(f"DONE in {time.time() - t0:.0f}s")
        return

    log("STEP 1/8: scaffold navpy wrapper + test + MsgType + ttl")
    scaffold_navpy(spec, args.dry_run)

    log("STEP 2/8: insert <message> into navlink.xml (WSL)")
    wsl_insert_xml(spec_path, args.dry_run)

    log("STEP 3/8: reinstall WSL pymavlink from local source")
    wsl_reinstall_pymavlink(spec, args.dry_run)

    log("STEP 4/8: copy navlink.xml + regen FULL ardupilotmega C headers (WSL)")
    wsl_regen_headers(args.dry_run)

    log("STEP 5/8: rebuild mavlink-router (WSL)")
    wsl_rebuild_router(args.dry_run)

    if args.skip_waf:
        log("STEP 6/8: ./waf plane SKIPPED (--skip-waf)")
    else:
        log("STEP 6/8: rebuild ArduPilot SITL ./waf plane (WSL, slow)")
        wsl_rebuild_sitl(args.dry_run)

    if args.no_push:
        log("STEP 7/8: fork push + Windows reinstall SKIPPED (--no-push)")
    else:
        log("STEP 7/8: push navlink.xml to fork + reinstall Windows .venv pymavlink")
        push_and_reinstall_windows(spec, venv_python, args.dry_run)

    log("STEP 8/8: CRC gate (WSL)")
    wsl_crc_gate(spec, args.dry_run)

    if args.no_route_test:
        log("routing proof SKIPPED (--no-route-test)")
    else:
        log("routing proof: launch swarm + send message (WSL)")
        wsl_route_test(spec, args.instances, args.dry_run)

    log(f"DONE in {time.time() - t0:.0f}s")
    log("Remaining manual step (deferred): --publish the c_library_v2 / ardupilot / "
        "mavlink-router submodule commits once the message stabilizes.")


if __name__ == "__main__":
    main()
