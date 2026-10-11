"""Classify import lines effective.py drops in kept modules (re-export vs unused; rule impact; side effects)."""
import csv, json, sys
eff = sys.argv[1]
rows = {r['src']: r for r in csv.DictReader(open('manifest.tsv', encoding='utf-8'), delimiter='\t')}
imp = json.load(open('imports.json'))['modules']
m2s = {v['module']: k for k, v in imp.items()}
rr = [r for r in csv.reader(open(f'{eff}/reexports.tsv', encoding='utf-8'), delimiter='\t') if r[0] == 'step1']
drops = [r for r in rr if r[3] == 'DROP-REEXPORT']
via = set((r[3], r[4]) for r in rr if r[3] != 'DROP-REEXPORT')
src = open('check_rules.py').read()
ns = {}
exec(src[src.index('def layer'):src.index('viol = []')], ns)
layer, allowed = ns['layer'], ns['allowed']
kept = [r for r in drops if rows[r[1]]['step1_dest'] != 'delete']
out = []
for _, s, line, _, name, target in kept:
    tsrc = m2s.get(target)
    kind = 'reexport' if (imp[s]['module'], name) in via else 'unused'
    bad = bool(tsrc) and not allowed(layer(rows[s]['step1_dest']), layer(rows[tsrc]['step1_dest']))
    sub = bool(tsrc) and imp[tsrc]['module'].endswith('.' + name)
    out.append((s, int(line), name, target, kind, 'y' if bad else 'n', 'y' if sub else 'n'))
out = sorted(set(out))
print('kept-module drop lines', len({(o[0], o[1]) for o in out}), 'modules', len({o[0] for o in out}))
for k in ('reexport', 'unused'):
    sel = [o for o in out if o[4] == k]
    print(k, 'lines', len({(o[0], o[1]) for o in sel}), 'modules', len({o[0] for o in sel}))
print('violating if kept:')
for o in out:
    if o[5] == 'y':
        print('  ', o)
print('submodule bindings:')
for o in out:
    if o[6] == 'y':
        print('  ', o)
if len(sys.argv) > 2:
    with open(sys.argv[2], 'w', encoding='utf-8', newline='') as fh:
        w = csv.writer(fh, delimiter='\t', lineterminator='\n')
        w.writerow(['src', 'line', 'name', 'from_module', 'kind', 'wrong_way_if_kept'])
        for o in out:
            w.writerow(o[:6])
