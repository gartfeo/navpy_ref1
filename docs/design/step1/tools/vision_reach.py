"""Pure-vision import guard on the effective post-step graph.

Usage: python vision_reach.py <eff_imports.json> <eff_manifest.tsv> <step1|final> [exceptions.tsv]
A flagged file (vision_path=y, dest under core/ or plugins/) must not reach a forbidden target at
runtime (TYPE_CHECKING imports skipped), following imports through any core/plugins file.
The walk stops at named exception edges (step1_exceptions.tsv rows with guard=vision) and at
forbidden targets. Prints each flagged file that reaches a forbidden target and one path.
"""
import csv, json, sys
from collections import deque

imp = json.load(open(sys.argv[1]))["modules"]
stage = sys.argv[3]
col = "final_dest" if stage == "final" else "step1_dest"
rows = {r["src"]: r for r in csv.DictReader(open(sys.argv[2], encoding="utf-8"), delimiter="\t")}
exc = set()
if len(sys.argv) > 4:
    for r in csv.DictReader(open(sys.argv[4], encoding="utf-8"), delimiter="\t"):
        if r["guard"] == "vision":
            exc.add((r["file"], r["imports"]))
FORBIDDEN = ("core/perception/geo/", "core/world/", "sim/", "core/approach/geo/",
             "core/skills/geo_approach/", "plugins/laws/geo/")


def d(k):
    return rows[k][col]


def walkable(k):
    return d(k).startswith(("core/", "plugins/"))


def forbidden(k):
    return d(k).startswith(FORBIDDEN)


edges = {}
for k, v in imp.items():
    ts = []
    for e in v["imports"]:
        if e["type_checking"]:
            continue
        t = e["target"].split("::", 1)[1]
        if t in rows and t != k:
            ts.append((t, e["line"], e["where"]))
    edges[k] = ts

flagged = sorted(k for k in rows if rows[k]["vision_path"] == "y" and walkable(k) and not forbidden(k))
bad = []
for f in flagged:
    prev = {f: None}
    q = deque([f])
    hit = None
    while q and not hit:
        u = q.popleft()
        for t, line, where in edges.get(u, []):
            if (d(u), d(t)) in exc or t in prev:
                continue
            prev[t] = (u, line, where)
            if forbidden(t):
                hit = t
                break
            if walkable(t):
                q.append(t)
    if hit:
        path, n = [], hit
        while prev[n]:
            u, line, where = prev[n]
            path.append(f"{d(u)}:{line}{'(fn)' if where == 'function' else ''}")
            n = u
        bad.append((f, list(reversed(path)) + [d(hit)]))
print(f"stage={stage} flagged={len(flagged)} reaching_forbidden={len(bad)} exceptions={len(exc)}")
for f, p in bad:
    print("  " + " -> ".join(p))
# every frontier edge: a file reachable from a flagged file (without crossing an exception) -> forbidden target
reach_from = {}
for f in flagged:
    seen = {f}
    q = deque([f])
    while q:
        u = q.popleft()
        for t, line, where in edges.get(u, []):
            if (d(u), d(t)) in exc:
                continue
            if forbidden(t):
                reach_from.setdefault((d(u), line, where, d(t)), set()).add(f)
                continue
            if t not in seen and walkable(t):
                seen.add(t)
                q.append(t)
print(f"frontier edges: {len(reach_from)}")
for (u, line, where, t), fs in sorted(reach_from.items()):
    print(f"  {u}:{line}{'(fn)' if where == 'function' else ''} -> {t}   [{len(fs)} flagged]")
# a named exception that no flagged file reaches any more is stale (the ratchet makes it fail)
reached = set()
for f in flagged:
    seen, q = {f}, deque([f])
    while q:
        u = q.popleft()
        for t, line, where in edges.get(u, []):
            if (d(u), d(t)) in exc:
                reached.add((d(u), d(t)))
                continue
            if t not in seen and walkable(t) and not forbidden(t):
                seen.add(t)
                q.append(t)
stale = sorted(exc - reached)
print(f"stale exceptions: {len(stale)}")
for e in stale:
    print("  ", e)
