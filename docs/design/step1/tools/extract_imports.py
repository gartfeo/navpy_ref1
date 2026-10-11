"""Build the navpy import graph for the step-1 re-plan.

Usage: python extract_imports.py <repo_root> <out.json>
Output: {"modules": {path: {...}}, "external_refs": {module: {area: count}}}
"""
import ast, json, os, sys
from collections import defaultdict

root = os.path.abspath(sys.argv[1]); out = sys.argv[2]
src = os.path.join(root, "src")
navpy_dir = os.path.join(src, "navpy")

def mod_name(path, base):
    rel = os.path.relpath(path, base).replace(os.sep, "/")[:-3]
    parts = rel.split("/")
    if parts[-1] == "__init__":
        parts = parts[:-1]
    return ".".join(parts)

modules = {}  # dotted -> rel path under src/navpy
for d, _, fs in os.walk(navpy_dir):
    if "__pycache__" in d:
        continue
    for f in fs:
        if f.endswith(".py"):
            p = os.path.join(d, f)
            modules[mod_name(p, src)] = os.path.relpath(p, navpy_dir).replace(os.sep, "/")

def resolve(target):
    """Longest prefix of target that is a navpy module."""
    parts = target.split(".")
    while parts:
        cand = ".".join(parts)
        if cand in modules:
            return cand
        parts.pop()
    return None

def scan(path, pkg):
    try:
        tree = ast.parse(open(path, encoding="utf-8").read())
    except Exception as e:
        return [], str(e), 0
    edges = []
    tc_lines = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.If):
            t = node.test
            name = getattr(t, "id", None) or getattr(t, "attr", None)
            if name == "TYPE_CHECKING":
                for n in ast.walk(node):
                    if hasattr(n, "lineno"):
                        tc_lines.add(n.lineno)
    top_ids = {id(n) for n in tree.body}
    def visit(node, depth):
        for child in ast.iter_child_nodes(node):
            nd = depth + (1 if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) else 0)
            if isinstance(child, (ast.Import, ast.ImportFrom)):
                targets = []
                if isinstance(child, ast.Import):
                    targets = [a.name for a in child.names]
                else:
                    if child.level:
                        base = pkg.split(".")
                        base = base[: len(base) - child.level + 1] if child.level > 1 else base
                        modpart = ".".join(base + ([child.module] if child.module else []))
                    else:
                        modpart = child.module or ""
                    targets = [modpart + "." + a.name for a in child.names] or [modpart]
                for t in targets:
                    if not t.startswith("navpy"):
                        continue
                    r = resolve(t)
                    edges.append({"line": child.lineno, "target": r or t,
                                  "resolved": r is not None,
                                  "where": "function" if depth else "top",
                                  "type_checking": child.lineno in tc_lines})
            visit(child, nd)
    visit(tree, 0)
    strings = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and "navpy." in node.value:
            strings += 1
    return edges, None, strings

result = {}
for name, rel in sorted(modules.items(), key=lambda kv: kv[1]):
    path = os.path.join(navpy_dir, rel)
    pkg = name if rel.endswith("__init__.py") else name.rsplit(".", 1)[0]
    edges, err, strings = scan(path, pkg)
    # dedupe same target per line
    seen = set(); ded = []
    for e in edges:
        k = (e["line"], e["target"])
        if k not in seen:
            seen.add(k); ded.append(e)
    result[rel] = {"module": name, "lines": sum(1 for _ in open(path, encoding="utf-8", errors="ignore")),
                   "imports": ded, "navpy_string_literals": strings, "parse_error": err}

ext = defaultdict(lambda: defaultdict(int))
for area in ["src/gcs", "scripts", "tests", "tools"]:
    base = os.path.join(root, area)
    for d, _, fs in os.walk(base):
        if "__pycache__" in d or "node_modules" in d:
            continue
        for f in fs:
            if not f.endswith(".py"):
                continue
            p = os.path.join(d, f)
            edges, _, _ = scan(p, "")
            for e in edges:
                if e["resolved"]:
                    ext[modules[e["target"]]][area] += 1
json.dump({"commit": os.popen(f'git -C "{root}" rev-parse --short HEAD').read().strip(),
           "modules": result, "external_refs": ext}, open(out, "w"), indent=1)
n_edges = sum(len(v["imports"]) for v in result.values())
unres = sum(1 for v in result.values() for e in v["imports"] if not e["resolved"])
print(f"{len(result)} files, {n_edges} navpy import edges, {unres} unresolved, {sum(1 for v in result.values() if v['parse_error'])} parse errors")
