"""c01 strip list: names a kept module re-exports across folders for its importers, plus unused names that
would be wrong-way after the move. Vendored files are never edited.
Usage: python make_strip_list.py <drops.tsv from classify_drops.py> <out.tsv>"""
import csv, json, os, sys
rows = {r['src']: r for r in csv.DictReader(open('manifest.tsv', encoding='utf-8'), delimiter='\t')}
imp = json.load(open('imports.json'))['modules']
m2s = {v['module']: k for k, v in imp.items()}
VENDORED = {'modules/vision/peripheral/siyi/siyi_sdk.py', 'modules/vision/peripheral/siyi/siyi_message.py',
            'modules/vehicle/_mavftp/_upstream_mavftp.py'}
ext = json.load(open('work/external_imports.json'))
splits = json.load(open('step1_splits.json'))


def home(o, name):
    for part in splits.get(o, {}).get('parts', []):
        if name in part['symbols']:
            return part['step1']
    return rows[o]['step1_dest']
out = []
for r in csv.DictReader(open(sys.argv[1], encoding='utf-8'), delimiter='\t'):
    if r['src'] in VENDORED:
        continue
    o = m2s.get(r['from_module'])
    cross = o is not None and os.path.dirname(rows[o]['step1_dest']) != os.path.dirname(rows[r['src']]['step1_dest'])
    users = ext.get(imp[r['src']]['module'] + '|' + r['name'], [])
    is_init = r['src'].endswith('__init__.py')
    if r['wrong_way_if_kept'] == 'y':
        why = 'wrong-way after the move'
    elif r['kind'] == 'reexport' or users:
        why = 're-export'
    elif is_init and cross:
        why = 'package-init re-export across folders'
    else:
        continue
    who = ('navpy; ' if r['kind'] == 'reexport' else '') + (f'{len(users)} outside' if users else '')
    why += f" ({who.strip('; ') or 'no importers'}); origin {home(o, r['name']) if o else r['from_module']}"
    out.append((r['src'], r['line'], r['name'], r['from_module'], why))
with open(sys.argv[2], 'w', encoding='utf-8', newline='') as fh:
    w = csv.writer(fh, delimiter='\t', lineterminator='\n')
    w.writerow(['src', 'line', 'name', 'imported_from', 'why'])
    w.writerows(out)
print('names', len(out), 'lines', len({(o[0], o[1]) for o in out}), 'modules', len({o[0] for o in out}))
for o in out:
    print('  ', o[0], o[1], o[2], '|', o[4][:90])
