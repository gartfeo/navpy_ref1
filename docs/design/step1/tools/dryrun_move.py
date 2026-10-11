"""Scratch dry-run of the step-1 moves (moves only; split files move whole; facades kept if still imported).

Usage: python dryrun_move.py <worktree> <manifest.tsv> <out_dir>
"""
import ast, csv, json, os, re, subprocess, sys
from collections import defaultdict

wt, manifest_path, out = sys.argv[1:4]
os.makedirs(out, exist_ok=True)
NAV = os.path.join(wt, "src", "navpy")
SCAN_DIRS = ["src", "tests", "scripts", "tools"]

rows = list(csv.DictReader(open(manifest_path, encoding="utf-8"), delimiter="\t"))


def git(*a):
    return subprocess.run(["git", "-C", wt, *a], capture_output=True, text=True, check=True).stdout


def modname(rel, prefix="navpy"):
    p = rel[:-3] if rel.endswith(".py") else rel
    parts = p.split("/")
    if parts[-1] == "__init__":
        parts = parts[:-1]
    return ".".join([prefix] + parts) if parts else prefix


log = defaultdict(list)
moves = []        # (old_rel_under_src, new_rel_under_src)
mod_map = {}      # old dotted -> new dotted
path_map = {}     # "navpy/old.py" -> "navpy/new.py" (path-string form, also gcs)
deletes = []
skipped = []
for r in rows:
    src, dest, act = r["src"].strip(), r["step1_dest"].strip(), r["action"].strip()
    if act in ("stay",):
        continue
    if act == "delete":
        deletes.append(src)
        continue
    if act == "split":
        skipped.append(src)  # moved whole to its first-listed (remaining-part) step-1 dest
    if dest.startswith("gcs:"):
        new_rel = "gcs/" + dest[4:]
        new_mod = modname(dest[4:], "gcs")
    else:
        new_rel = "navpy/" + dest
        new_mod = modname(dest)
    old_rel = "navpy/" + src
    moves.append((old_rel, new_rel))
    path_map[old_rel] = new_rel
    if src.endswith(".py"):
        mod_map[modname(src)] = new_mod

# delete-row modules stop prefix rewriting (identity), so a shim under a moved package keeps its old name
for src in deletes:
    if src.endswith(".py"):
        mod_map.setdefault(modname(src), modname(src))
mod_map = {k: v for k, v in mod_map.items()}

# 1. git mv
for old, new in moves:
    os.makedirs(os.path.dirname(os.path.join(wt, "src", new)), exist_ok=True)
    git("mv", os.path.join("src", old), os.path.join("src", new))

# 2. create missing __init__.py for every new package dir under src/navpy and src/gcs
created_inits = []
for old, new in moves:
    d = os.path.dirname(os.path.join(wt, "src", new))
    top = os.path.join(wt, "src", new.split("/")[0])
    while os.path.normpath(d) != os.path.normpath(top):
        ini = os.path.join(d, "__init__.py")
        if not os.path.exists(ini):
            open(ini, "w").close()
            created_inits.append(os.path.relpath(ini, wt).replace("\\", "/"))
        d = os.path.dirname(d)

# 3. rewrite dotted references and path strings
keys_sorted = sorted(mod_map, key=len, reverse=True)
DOT = re.compile(r"(?<![\w.])navpy(?:\.[A-Za-z_][A-Za-z0-9_]*)+")
path_keys = sorted(path_map, key=len, reverse=True)
PATHRE = re.compile("|".join(re.escape(k) for k in path_keys))


def rewrite_dotted(m):
    s = m.group(0)
    parts = s.split(".")
    for n in range(len(parts), 1, -1):
        pre = ".".join(parts[:n])
        if pre in mod_map:
            return ".".join([mod_map[pre]] + parts[n:])
    return s


def py_files():
    for d in SCAN_DIRS:
        base = os.path.join(wt, d)
        for root, dirs, files in os.walk(base):
            dirs[:] = [x for x in dirs if x not in ("node_modules", "__pycache__", ".git")]
            for f in files:
                if f.endswith(".py"):
                    yield os.path.join(root, f)


def fix_from_submodules(text, fpath):
    """from <pkg> import <submodule> where <pkg>.<submodule> moved: rewrite the statement."""
    try:
        tree = ast.parse(text)
    except SyntaxError as e:
        log["syntax_error"].append(f"{fpath}: {e}")
        return text, 0
    edits = []
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.level == 0 and n.module and n.module.split(".")[0] == "navpy":
            hits = [a for a in n.names if f"{n.module}.{a.name}" in mod_map]
            if not hits:
                continue
            rest = [a for a in n.names if a not in hits]
            lines = text.splitlines(keepends=True)
            indent = re.match(r"\s*", lines[n.lineno - 1]).group(0)
            new = []
            if rest:
                new.append(indent + f"from {n.module} import " + ", ".join(
                    a.name + (f" as {a.asname}" if a.asname else "") for a in rest))
            for a in hits:
                nm = mod_map[f"{n.module}.{a.name}"]
                if nm == f"{n.module}.{a.name}":  # kept in place (identity): spell the full name
                    new.append(indent + f"import {nm} as {a.asname or a.name}")
                    continue
                parent, leaf = nm.rsplit(".", 1)
                new.append(indent + f"from {parent} import {leaf}" + (f" as {a.asname}" if a.asname else
                                                                     (f" as {a.name}" if leaf != a.name else "")))
            edits.append((n.lineno, n.end_lineno, "\n".join(new) + "\n"))
    if not edits:
        return text, 0
    lines = text.splitlines(keepends=True)
    for s, e, rep in sorted(edits, reverse=True):
        lines[s - 1:e] = [rep]
    return "".join(lines), len(edits)



# 3a. facade resolution (c01-lite): importers of delete-row modules name the defining module instead.
BASE = sys.argv[4] if len(sys.argv) > 4 else None
DEL_MODS = {modname(x) for x in deletes if x.endswith(".py")}
_ast_cache = {}


def _base_file(mod):
    rel = mod.split(".")
    for cand in (os.path.join(BASE, "src", *rel) + ".py", os.path.join(BASE, "src", *rel, "__init__.py")):
        if os.path.exists(cand):
            return cand
    return None


def _scan(mod):
    if mod in _ast_cache:
        return _ast_cache[mod]
    f = _base_file(mod)
    defs, binds = set(), {}
    if f:
        tree = ast.parse(open(f, encoding="utf-8").read())
        is_pkg = f.endswith("__init__.py")
        pkg = mod if is_pkg else mod.rsplit(".", 1)[0]
        nodes = []
        for n in tree.body:
            nodes.append(n)
            if isinstance(n, (ast.If, ast.Try)):
                # only TYPE_CHECKING-free top-level branches; include all statements of if/try bodies
                for b in (n.body + getattr(n, "orelse", []) + getattr(n, "finalbody", [])
                          + [x for h in getattr(n, "handlers", []) for x in h.body]):
                    nodes.append(b)
        for x in nodes:
            if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                defs.add(x.name)
            elif isinstance(x, (ast.Assign, ast.AnnAssign)):
                for g in (x.targets if isinstance(x, ast.Assign) else [x.target]):
                    for nn in ast.walk(g):
                        if isinstance(nn, ast.Name):
                            defs.add(nn.id)
            elif isinstance(x, ast.ImportFrom):
                if x.level:
                    base = pkg.split(".")
                    if x.level > 1:
                        base = base[: len(base) - x.level + 1]
                    t = ".".join(base + ([x.module] if x.module else []))
                else:
                    t = x.module
                for a in x.names:
                    binds[a.asname or a.name] = (t, a.name)
    _ast_cache[mod] = (defs, binds, f is not None)
    return _ast_cache[mod]


def resolve(mod, name, depth=0):
    """-> ("name", origin_mod, origin_name) | ("module", dotted) | None"""
    if depth > 12:
        return None
    defs, binds, exists = _scan(mod)
    if not exists:
        return None
    if name in binds and name not in defs:
        t, o = binds[name]
        if t and t.split(".")[0] == "navpy":
            if _base_file(f"{t}.{o}"):
                return ("module", f"{t}.{o}")
            r = resolve(t, o, depth + 1)
            return r if r else ("name", t, o)
        return None  # external re-export: keep
    if name in defs:
        return ("name", mod, name)
    if _base_file(f"{mod}.{name}"):
        return ("module", f"{mod}.{name}")
    return None


facade_log = defaultdict(int)
facade_unresolved = []


def fix_facades(text, fpath):
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return text, 0
    edits = []
    for n in ast.walk(tree):
        if not (isinstance(n, ast.ImportFrom) and n.level == 0 and n.module in DEL_MODS):
            continue
        out_lines, keep = [], []
        for a in n.names:
            r = resolve(n.module, a.name)
            if r is None or (r[0] == "name" and r[1] == n.module):
                keep.append(a)
                facade_unresolved.append(f"{os.path.relpath(fpath, wt)}:{n.lineno} {n.module}.{a.name}")
                continue
            alias = a.asname or a.name
            if r[0] == "module":
                parent, leaf = r[1].rsplit(".", 1)
                out_lines.append(f"from {parent} import {leaf}" + (f" as {alias}" if alias != leaf else ""))
            else:
                out_lines.append(f"from {r[1]} import {r[2]}" + (f" as {alias}" if alias != r[2] else ""))
            facade_log[n.module] += 1
        if not out_lines:
            continue
        lines = text.splitlines(keepends=True)
        indent = re.match(r"\s*", lines[n.lineno - 1]).group(0)
        if keep:
            out_lines.insert(0, f"from {n.module} import " + ", ".join(
                a.name + (f" as {a.asname}" if a.asname else "") for a in keep))
        edits.append((n.lineno, n.end_lineno, "".join(indent + l + "\n" for l in out_lines)))
    if not edits:
        return text, 0
    lines = text.splitlines(keepends=True)
    for s_, e_, rep in sorted(edits, reverse=True):
        lines[s_ - 1:e_] = [rep]
    return "".join(lines), len(edits)


facade_hits = 0
changed_files = 0
dotted_hits = 0
path_hits = 0
from_sub_hits = 0
for fp in py_files():
    try:
        text = open(fp, encoding="utf-8").read()
    except UnicodeDecodeError:
        log["undecodable"].append(fp)
        continue
    orig = text
    if BASE:
        text, nf = fix_facades(text, fp)
        facade_hits += nf
    text, n1 = fix_from_submodules(text, fp)
    n2 = sum(1 for x in DOT.findall(text) if rewrite_dotted(re.match(".*", x)) != x)
    text = DOT.sub(rewrite_dotted, text)
    text, n3 = PATHRE.subn(lambda m: path_map[m.group(0)], text)
    if text != orig:
        open(fp, "w", encoding="utf-8", newline="").write(text)
        changed_files += 1
        dotted_hits += n2
        path_hits += n3
        from_sub_hits += n1

# 4. deletes: only if nothing references the old module any more (iterate to a fixpoint)
all_text = {os.path.normpath(fp): open(fp, encoding="utf-8").read() for fp in py_files()}
deleted, pending = [], list(deletes)
changed = True
users_of = {}
while changed:
    changed = False
    for src in list(pending):
        mod = modname(src)
        own = os.path.normpath(os.path.join(NAV, src))
        pat = re.compile(r"(?<![\w.])" + re.escape(mod) + r"(?![\w])")
        users = [os.path.relpath(fp, wt).replace("\\", "/") for fp, t in all_text.items()
                 if fp != own and pat.search(t)]
        users_of[src] = users
        if not users:
            git("rm", "-q", "-f", os.path.join("src", "navpy", src))
            all_text.pop(own, None)
            deleted.append(src)
            pending.remove(src)
            changed = True
kept = [(s, f"{len(users_of[s])} users: " + ", ".join(users_of[s][:8])) for s in pending]
# an __init__ whose folder still holds kept files stays (restore it)
for src in list(deleted):
    if src.endswith("__init__.py"):
        d = os.path.dirname(os.path.join(NAV, src))
        if os.path.isdir(d) and any(f.endswith(".py") for f in os.listdir(d)):
            git("checkout", "HEAD", "--", os.path.join("src", "navpy", src))
            deleted.remove(src)
            kept.append((src, "folder still holds kept files"))

# 5. leftover non-module references to old paths (dirs / non-py files)
leftover = []
old_dirs = sorted({"src/navpy/" + os.path.dirname(r["src"]) for r in rows if "/" in r["src"]}, key=len, reverse=True)
for d in ("src", "tests", "scripts", "tools"):
    pass

summary = {
    "moves": len(moves), "skipped_splits_moved_whole": skipped, "created_inits": created_inits,
    "files_rewritten": changed_files, "dotted_rewrites": dotted_hits, "path_rewrites": path_hits,
    "from_pkg_import_submodule_rewrites": from_sub_hits,
    "facade_import_rewrites": facade_hits, "facade_by_module": facade_log, "facade_unresolved": facade_unresolved, "deleted": deleted, "kept_deletes": kept, "log": log,
}
json.dump(summary, open(os.path.join(out, "move_summary.json"), "w"), indent=1)
json.dump(mod_map, open(os.path.join(out, "mod_map.json"), "w"), indent=1)
print(json.dumps({k: (len(v) if isinstance(v, (list, dict)) else v) for k, v in summary.items()}, indent=1))
for k in kept:
    print("KEPT", *k)
