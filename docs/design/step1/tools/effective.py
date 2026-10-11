"""Effective import graph after step-1 facade deletion, re-export removal and pure-move splits.

Usage: python effective.py <src/navpy dir> <imports.json> <manifest.tsv> <splits.json> <out_dir>
Writes out_dir/eff_imports.json and out_dir/eff_manifest.tsv (virtual rows for split parts,
"<src>#<part dest basename>") so the unchanged check_rules.py can judge the real post-move edges.
Also writes out_dir/reexports.tsv (importer rewrites the move tool must do) and
out_dir/unresolved.tsv (names whose origin is a deleted definition).
"""
import ast, csv, json, os, re, sys
from collections import defaultdict

root, imports_path, manifest_path, splits_path, out = sys.argv[1:6]
# Importers always name the defining module (every re-export is followed).
# Optional strip list (c01): TSV with src and name columns. With it, an unused import in a kept module is
# dropped only when listed; other unused imports stay as edges. Without it, every unused one is dropped.
STRIP = None
if len(sys.argv) > 6:
    with open(sys.argv[6], encoding="utf-8") as fh:
        STRIP = {(r["src"], r["name"]) for r in csv.DictReader(fh, delimiter="	")}
imports = json.load(open(imports_path))["modules"]
rows = {}
with open(manifest_path, encoding="utf-8") as fh:
    for r in csv.DictReader(fh, delimiter="\t"):
        rows[r["src"].strip()] = r
splits = json.load(open(splits_path))
mod_to_src = {v["module"]: k for k, v in imports.items()}
REMOVED_NAMES = {"DetectorAbc"}  # behaviorless compat marker; the base is dropped with detector_abc.py


def parse(src):
    return ast.parse(open(os.path.join(root, src), encoding="utf-8").read())


trees = {s: parse(s) for s in imports}


def pkg_of(src):
    m = imports[src]["module"]
    return m if src.endswith("__init__.py") else m.rsplit(".", 1)[0]


def abs_module(node, src):
    if node.level:
        base = pkg_of(src).split(".")
        if node.level > 1:
            base = base[: len(base) - node.level + 1]
        return ".".join(base + ([node.module] if node.module else []))
    return node.module or ""


def tc_lines(tree):
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.If):
            t = n.test
            if (getattr(t, "id", None) or getattr(t, "attr", None)) == "TYPE_CHECKING":
                for c in ast.walk(n):
                    if hasattr(c, "lineno"):
                        out.add(c.lineno)
    return out


# top-level definitions and import bindings per module
defs = {}      # src -> set(names defined at top level)
binds = {}     # src -> {bound_name: (target_module, orig_name or None for module import)}
for s, t in trees.items():
    d, b = set(), {}
    for n in t.body:
        stmts = [n]
        if isinstance(n, (ast.If, ast.Try)):
            stmts = list(ast.walk(n))
        for x in stmts:
            if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and x in t.body:
                d.add(x.name)
            elif isinstance(x, (ast.Assign, ast.AnnAssign)) and (x in t.body):
                tg = x.targets if isinstance(x, ast.Assign) else [x.target]
                for g in tg:
                    for nn in ast.walk(g):
                        if isinstance(nn, ast.Name):
                            d.add(nn.id)
            elif isinstance(x, ast.ImportFrom):
                m = abs_module(x, s)
                for a in x.names:
                    b[a.asname or a.name] = (m, a.name)
            elif isinstance(x, ast.Import):
                for a in x.names:
                    b[a.asname or a.name.split(".")[0]] = (a.name, None)
    defs[s], binds[s] = d, b


def resolve_mod(dotted):
    parts = dotted.split(".")
    while parts:
        c = ".".join(parts)
        if c in mod_to_src:
            return mod_to_src[c]
        parts.pop()
    return None


def split_part(src, name, stage):
    sp = splits.get(src)
    if not sp:
        return None
    for p in sp["parts"]:
        if name in p["symbols"]:
            return p[stage]
    return None


def origin(src, name, seen=None):
    """(defining src, name) following re-exports."""
    seen = seen or set()
    if (src, name) in seen:
        return src, name
    seen.add((src, name))
    if name in defs[src]:
        return src, name
    b = binds[src].get(name)
    if b and b[0].startswith("navpy"):
        m, orig = b
        if orig is None:
            return src, name
        sub = m + "." + orig
        if sub in mod_to_src:  # bound a submodule
            return mod_to_src[sub], None
        t = resolve_mod(m)
        if t:
            return origin(t, orig, seen)
    return src, name


def dest(src, stage):
    return rows[src]["final_dest" if stage == "final" else "step1_dest"].strip()


def vkey(src, d):
    return src if d is None else f"{src}#{d}"


def part_dest(src, name, stage):
    if name is None:
        return None
    return split_part(src, name, stage)


# usage of bound names by top-level statements (for split sources and dead re-export lines).
# Real references only: Name nodes, plus names inside string annotations (forward references).
# Docstrings and other string constants are not references.
def _annotation_nodes(node):
    for n in ast.walk(node):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            a = n.args
            for arg in a.posonlyargs + a.args + a.kwonlyargs + [a.vararg, a.kwarg]:
                if arg is not None and arg.annotation is not None:
                    yield arg.annotation
            if n.returns is not None:
                yield n.returns
        elif isinstance(n, ast.AnnAssign):
            yield n.annotation


def names_used(node):
    used = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Name):
            used.add(n.id)
    for ann in _annotation_nodes(node):
        for c in ast.walk(ann):
            if isinstance(c, ast.Constant) and isinstance(c.value, str):
                try:
                    expr = ast.parse(c.value, mode="eval")
                except SyntaxError:
                    continue
                used.update(x.id for x in ast.walk(expr) if isinstance(x, ast.Name))
    return used


def stmt_symbol(n):
    if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return [n.name]
    if isinstance(n, (ast.Assign, ast.AnnAssign)):
        tg = n.targets if isinstance(n, ast.Assign) else [n.target]
        return [nn.id for g in tg for nn in ast.walk(g) if isinstance(nn, ast.Name)]
    return []


_ubp_cache = {}


def users_by_part(src, stage):
    key = (src, stage)
    if key not in _ubp_cache:
        _ubp_cache[key] = _users_by_part(src, stage)
    return _ubp_cache[key]


def _users_by_part(src, stage):
    """bound name -> set(part dest or None) of top-level statements that use it (not __all__)."""
    t = trees[src]
    res = defaultdict(set)
    for n in t.body:
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            continue
        if isinstance(n, ast.If) and all(isinstance(c, (ast.Import, ast.ImportFrom)) for c in n.body):
            continue
        syms = stmt_symbol(n)
        if syms == ["__all__"]:
            continue
        part = None
        for sy in syms:
            p = split_part(src, sy, stage)
            if p:
                part = p
        for u in names_used(n):
            res[u].add(part)
    return res


eff_mods = {}
rewrites = []
unresolved = []
for stage in ("final", "step1"):
    pass

stage_out = {}
for stage in ("final", "step1"):
    emods = defaultdict(lambda: {"imports": []})
    for s, info in imports.items():
        tcl = tc_lines(trees[s])
        usage = users_by_part(s, stage) if (s in splits) else None
        all_usage = None
        # function-level imports are inside statements -> attribute by walking
        for node in ast.walk(trees[s]):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            edge = next((e for e in info["imports"] if e["line"] == node.lineno), None)
            if edge is None:
                continue
            where = edge["where"]
            tc = node.lineno in tcl
            # which part of s owns this import
            if s in splits:
                owners = None
                if where == "function":
                    # find enclosing top-level statement
                    for top in trees[s].body:
                        if top.lineno <= node.lineno <= getattr(top, "end_lineno", top.lineno):
                            p = None
                            for sy in stmt_symbol(top):
                                p = split_part(s, sy, stage) or p
                            owners = {p}
            targets = []  # (target src, part dest, bound name)
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name.startswith("navpy"):
                        t = resolve_mod(a.name)
                        if t:
                            targets.append((t, None, a.asname or a.name.split(".")[0], a.name))
            else:
                m = abs_module(node, s)
                if not m.startswith("navpy"):
                    continue
                for a in node.names:
                    sub = m + "." + a.name
                    if sub in mod_to_src:
                        targets.append((mod_to_src[sub], None, a.asname or a.name, None))
                        continue
                    t = resolve_mod(m)
                    if not t:
                        continue
                    o, oname = origin(t, a.name)
                    if oname in REMOVED_NAMES:
                        continue
                    if o != t:
                        rewrites.append((stage, s, node.lineno, imports[t]["module"], a.name, imports[o]["module"]))
                    od = dest(o, stage)
                    if od == "delete" and oname is not None:
                        unresolved.append((stage, s, node.lineno, imports[o]["module"], oname))
                    targets.append((o, part_dest(o, oname, stage), a.asname or a.name, oname))
            for (t, pd, bound, oname) in targets:
                if s in splits:
                    if where == "function":
                        own = owners
                    else:
                        own = usage.get(bound, set()) if usage is not None else {None}
                    if not own:
                        # unused by any part: a re-export or dead import, dropped with the split
                        continue
                else:
                    own = {None}
                    # dead re-export line in a kept module: bound name used nowhere but __all__
                    if where == "top" and not tc:
                        ub = users_by_part(s, stage)
                        if (bound not in ub and isinstance(node, ast.ImportFrom)
                                and (STRIP is None or (s, bound) in STRIP)):
                            # pure re-export: importers were rewritten to the origin, line dropped
                            rewrites.append((stage, s, node.lineno, "DROP-REEXPORT", bound, imports[t]["module"]))
                            continue
                for p in own:
                    src_key = vkey(s, p)
                    tgt_key = vkey(t, pd)
                    if src_key == tgt_key:
                        continue
                    emods[src_key]["imports"].append({"line": node.lineno, "target": "navpy.V::" + tgt_key,
                                                      "resolved": True, "where": where, "type_checking": tc})
        # intra-file references between split parts
        if s in splits:
            for n in trees[s].body:
                syms = stmt_symbol(n)
                if syms == ["__all__"]:
                    continue
                p = None
                for sy in syms:
                    p = split_part(s, sy, stage) or p
                for u in names_used(n):
                    if u in defs[s] and u not in syms:
                        q = split_part(s, u, stage)
                        if q != p:
                            emods[vkey(s, p)]["imports"].append({"line": n.lineno, "target": "navpy.V::" + vkey(s, q),
                                                                 "resolved": True, "where": "top", "type_checking": False})
    # make sure every module and part exists
    for s in imports:
        emods[vkey(s, None)]
        for p in splits.get(s, {}).get("parts", []):
            emods[vkey(s, p[stage])]
    for k, v in emods.items():
        v["module"] = "navpy.V::" + k
    # dedupe
    for v in emods.values():
        seen, ded = set(), []
        for e in v["imports"]:
            kk = (e["line"], e["target"])
            if kk not in seen:
                seen.add(kk)
                ded.append(e)
        v["imports"] = ded
    stage_out[stage] = dict(emods)

os.makedirs(out, exist_ok=True)
for stage in ("final", "step1"):
    json.dump({"modules": stage_out[stage]}, open(os.path.join(out, f"eff_imports_{stage}.json"), "w"), indent=0)
    with open(os.path.join(out, f"eff_manifest_{stage}.tsv"), "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh, delimiter="\t")
        w.writerow(["src", "final_dest", "step1_dest", "action", "vision_path", "note"])
        for k in stage_out[stage]:
            if "#" in k:
                s, d = k.split("#", 1)
                vp = next(p.get("vision_path", rows[s]["vision_path"]) for p in splits[s]["parts"] if p[stage] == d)
                w.writerow([k, d, d, "split-part", vp, ""])
            else:
                r = rows[k]
                w.writerow([k, r["final_dest"], r["step1_dest"], r["action"], r["vision_path"], ""])
with open(os.path.join(out, "reexports.tsv"), "w", encoding="utf-8", newline="") as fh:
    w = csv.writer(fh, delimiter="\t")
    w.writerow(["stage", "src", "line", "imported_from", "name", "origin"])
    w.writerows(sorted(set(rewrites)))
with open(os.path.join(out, "unresolved.tsv"), "w", encoding="utf-8", newline="") as fh:
    w = csv.writer(fh, delimiter="\t")
    w.writerow(["stage", "src", "line", "deleted_module", "name"])
    w.writerows(sorted(set(unresolved)))
print("rewrites", len({r for r in rewrites if r[0] == 'step1'}), "unresolved", len({u for u in unresolved if u[0] == 'step1'}))
