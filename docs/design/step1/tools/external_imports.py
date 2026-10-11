"""(module, name) pairs that code outside src/navpy imports with 'from navpy.<module> import name'.
Usage: python external_imports.py <repo root> <out.json>"""
import ast, json, os, sys
root = sys.argv[1]
pairs = {}
for top in ('tests', 'scripts', 'tools', 'src/gcs'):
    for dp, dn, fn in os.walk(os.path.join(root, top)):
        dn[:] = [d for d in dn if d not in ('node_modules', '__pycache__', '.venv')]
        for f in fn:
            if not f.endswith('.py'):
                continue
            p = os.path.join(dp, f)
            try:
                t = ast.parse(open(p, encoding='utf-8', errors='replace').read())
            except SyntaxError:
                continue
            for n in ast.walk(t):
                if isinstance(n, ast.ImportFrom) and n.module and n.module.startswith('navpy') and not n.level:
                    for a in n.names:
                        pairs.setdefault(n.module + '|' + a.name, []).append(os.path.relpath(p, root).replace(os.sep, '/'))
json.dump(pairs, open(sys.argv[2], 'w'), indent=0)
print('pairs', len(pairs))
