"""Check a step-1 manifest against the layout's import rules.

Usage: python check_rules.py <imports.json> <manifest.tsv> <final|step1> [--list out.tsv]
Manifest TSV (header row): src  final_dest  step1_dest  action  vision_path  note
  src/dest are paths under src/navpy/ (e.g. core/flow/loop.py); "gcs:<path>" = moves to src/gcs;
  "delete" = removed (shim/facade). Rows for every file in imports.json are required.
Layers from dest prefix: missions/<m>, core/kernel, core/ports, core/<x>, plugins/<family>[/<impl>],
  sim, app (also root main.py, main_gui.py, runtime_*.py if placed at root), gcs.
Rules: kernel->kernel; ports->kernel,ports; core/*->core/*; plugins/F->kernel,ports,F;
  missions/M->core/*,M; sim->core,plugins,sim; app->anything; nothing imports app/gcs except app.
Function-level and TYPE_CHECKING imports count, but are reported separately.
"""
import csv, json, sys
from collections import Counter, defaultdict

imports = json.load(open(sys.argv[1]))["modules"]
stage = sys.argv[3]
col = "final_dest" if stage == "final" else "step1_dest"
rows = {}
with open(sys.argv[2], encoding="utf-8") as fh:
    for r in csv.DictReader(fh, delimiter="\t"):
        rows[r["src"].strip()] = r

missing = sorted(set(imports) - set(rows))
extra = sorted(set(rows) - set(imports) - {"modules/vision/vision_profiles.json", "AGENTS.md", "CLAUDE.md"})
mod_to_src = {v["module"]: k for k, v in imports.items()}

def layer(dest):
    dest = (dest or "").strip()
    if not dest or dest == "delete":
        return ("deleted",)
    if dest.startswith("gcs:"):
        return ("gcs",)
    p = dest.split("/")
    if p[0] == "missions":
        return ("missions", p[1] if len(p) > 2 else "")
    if p[0] == "core":
        return ("core", p[1] if len(p) > 2 else "")
    if p[0] == "plugins":
        return ("plugins", p[1] if len(p) > 2 else "")
    if p[0] in ("sim", "app"):
        return (p[0],)
    if len(p) == 1:
        return ("app",)  # root entry points
    return ("unknown", p[0])

def allowed(a, b):
    if a[0] in ("app", "deleted"):
        return True
    if b[0] in ("app", "gcs"):
        return False
    if b[0] == "deleted":
        return False
    if a[0] == "core":
        if a[1] == "kernel":
            return b == ("core", "kernel")
        if a[1] == "ports":
            return b[0] == "core" and b[1] in ("kernel", "ports")
        return b[0] == "core"
    if a[0] == "plugins":
        return (b[0] == "core" and b[1] in ("kernel", "ports")) or b == a
    if a[0] == "missions":
        return b[0] == "core" or b == a
    if a[0] == "sim":
        return b[0] in ("core", "plugins", "sim")
    if a[0] == "gcs":
        return True
    return False

viol = []
for src, info in imports.items():
    r = rows.get(src)
    if not r:
        continue
    a = layer(r[col])
    for e in info["imports"]:
        tsrc = mod_to_src.get(e["target"])
        if tsrc is None or tsrc == src:
            continue
        tr = rows.get(tsrc)
        if not tr:
            continue
        b = layer(tr[col])
        if not allowed(a, b):
            viol.append((src, e["line"], tsrc, "/".join(a), "/".join(b), e["where"], e["type_checking"]))

print(f"stage={stage} rows={len(rows)} missing={len(missing)} extra={len(extra)}")
if missing:
    print("MISSING:", *missing[:40], sep="\n  ")
if extra:
    print("EXTRA:", *extra[:20], sep="\n  ")
dest_counts = Counter("/".join(layer(r[col])) for r in rows.values())
print("files per layer:", dict(sorted(dest_counts.items())))
top = [v for v in viol if v[5] == "top" and not v[6]]
print(f"violations: {len(viol)} edges (top-level {len(top)}, function-level {sum(1 for v in viol if v[5]=='function')}, TYPE_CHECKING {sum(1 for v in viol if v[6])}) in {len({v[0] for v in viol})} files")
pairs = Counter((v[3], v[4]) for v in viol)
for (a, b), n in pairs.most_common(40):
    print(f"  {n:4d}  {a} -> {b}")
if "--list" in sys.argv:
    with open(sys.argv[sys.argv.index("--list") + 1], "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh, delimiter="\t")
        w.writerow(["src", "line", "imports", "from_layer", "to_layer", "where", "type_checking"])
        w.writerows(sorted(viol))
