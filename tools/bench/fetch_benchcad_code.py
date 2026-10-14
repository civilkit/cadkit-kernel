"""Fetch the CadQuery programs of the BenchCAD parts in manifest.json (HF dataset BenchCAD/BenchCAD,
config code_gen, licence cc-by-4.0) into a scratch dir. Not stored in the repo.
usage: python3 fetch_benchcad_code.py manifest.json outdir"""
import json, sys, os, time, urllib.parse, urllib.request
from concurrent.futures import ThreadPoolExecutor

m = json.load(open(sys.argv[1]))
out = sys.argv[2]
os.makedirs(out, exist_ok=True)


def one(p):
    dst = f"{out}/{p['id']}.py"
    if os.path.exists(dst):
        return p['id'], 'cached'
    url = ("https://datasets-server.huggingface.co/filter?dataset=BenchCAD/BenchCAD&config=code_gen&split=code_gen&length=1&where="
           + urllib.parse.quote(f"\"stem\"='{p['benchcad_stem']}'"))
    for _ in range(6):
        try:
            d = json.load(urllib.request.urlopen(url, timeout=180))
            if d['rows']:
                open(dst, 'w').write(d['rows'][0]['row']['code'])
                return p['id'], 'ok'
            return p['id'], 'MISSING'
        except Exception:
            time.sleep(3)
    return p['id'], 'FAILED'


with ThreadPoolExecutor(12) as ex:
    for r in ex.map(one, m['parts']):
        print(*r, flush=True)
