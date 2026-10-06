#!/usr/bin/env python3
"""
Insert a navlink <message> block into navlink.xml from a message spec.

Runs on either OS but is normally invoked inside WSL by add_navlink_message.py
(the navlink.xml source of truth lives in the WSL ardupilot clone). Idempotent:
if a message with the same id or name already exists it is left untouched.

Usage:
    python3 navlink_xml_insert.py --xml <navlink.xml> --spec <spec.json>

Exit codes:
    0 - inserted, or already present (idempotent no-op)
    2 - conflict (id/name reused with different definition) or malformed xml
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape, quoteattr


def load_spec(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def render_message_block(spec):
    """Render the <message> ... </message> XML block from the spec.

    All text and attribute values are XML-escaped so arbitrary description/units
    strings (e.g. containing & < > ") cannot corrupt navlink.xml.
    """
    lines = [f'    <message id={quoteattr(str(spec["id"]))} name={quoteattr(spec["name"])}>']
    lines.append(f'      <description>{escape(spec["description"])}</description>')
    for fld in spec["fields"]:
        units = fld.get("units")
        unit_attr = f" units={quoteattr(str(units))}" if units else ""
        lines.append(
            f'      <field type={quoteattr(fld["type"])} name={quoteattr(fld["name"])}{unit_attr}>'
            f'{escape(fld["desc"])}</field>'
        )
    lines.append("    </message>")
    return "\n".join(lines)


def _spec_signature(spec):
    """Comparable identity of a spec message: (id, name, desc, ordered fields)."""
    fields = tuple(
        (f["name"], f["type"], (str(f["units"]) if f.get("units") else None))
        for f in spec["fields"]
    )
    return (int(spec["id"]), spec["name"], spec["description"].strip(), fields)


def _element_signature(elem):
    """Same identity extracted from an existing <message> element."""
    desc_el = elem.find("description")
    desc = (desc_el.text or "").strip() if desc_el is not None else ""
    fields = tuple(
        (fe.attrib.get("name"), fe.attrib.get("type"), fe.attrib.get("units"))
        for fe in elem.findall("field")
    )
    return (int(elem.attrib["id"]), elem.attrib["name"], desc, fields)


def _find_existing(xml_path, name, msg_id):
    """Return the <message> element matching name or id, or None."""
    root = ET.parse(xml_path).getroot()
    for msg in root.iter("message"):
        if msg.attrib.get("name") == name or int(msg.attrib.get("id", -1)) == msg_id:
            return msg
    return None


def main():
    ap = argparse.ArgumentParser(description="Insert a navlink message into navlink.xml")
    ap.add_argument("--xml", required=True, help="Path to navlink.xml")
    ap.add_argument("--spec", required=True, help="Path to message spec JSON")
    args = ap.parse_args()

    spec = load_spec(args.spec)
    name = spec["name"]
    msg_id = int(spec["id"])

    # Idempotency / conflict: an existing message counts as a no-op ONLY when its
    # full definition (id, name, description, ordered fields+units) matches the
    # spec. A same-id/name message with different fields is a CONFLICT (the CRC
    # would change), not a silent no-op.
    existing = _find_existing(args.xml, name, msg_id)
    if existing is not None:
        if _element_signature(existing) == _spec_signature(spec):
            print(f"[xml] {name} (id {msg_id}) already present, definition matches - no-op")
            return 0
        print(
            f"[xml] CONFLICT: a message with name '{existing.attrib.get('name')}' / "
            f"id {existing.attrib.get('id')} already exists but its definition differs "
            f"from the spec for {name}/{msg_id}. Remove/reconcile it in navlink.xml first.",
            file=sys.stderr,
        )
        return 2

    with open(args.xml, "r", encoding="utf-8") as f:
        content = f.read()

    block = render_message_block(spec)
    marker = "  </messages>"
    if marker not in content:
        print("[xml] ERROR: could not find '</messages>' marker", file=sys.stderr)
        return 2

    # Insert the block (blank line before it) right ahead of the closing tag.
    new_content = content.replace(marker, f"\n{block}\n{marker}", 1)
    with open(args.xml, "w", encoding="utf-8", newline="\n") as f:
        f.write(new_content)
    print(f"[xml] inserted {name} (id {msg_id}) into {args.xml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
