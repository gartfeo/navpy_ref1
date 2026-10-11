#!/usr/bin/env bash
# Re-run the step-1 analysis from manifest.tsv + step1_splits.json. Writes step1_reexports.tsv,
# step1_exceptions.tsv, final_violations.tsv and work/out/*. Planning prototype: paths are from the
# 2026-10-11 run (replan dir beside the wt-replan checkout); c00 replaces these tools.
set -euo pipefail
PY=${PY:-python}
SRC=../wt-replan/src/navpy
OUT=work/out
rm -rf "$OUT" work/out_pass1
mkdir -p "$OUT" work/out_pass1
echo "== plain checker (split files as one unit)"
$PY check_rules.py imports.json manifest.tsv step1 | sed -n '1p;4p'
$PY check_rules.py imports.json manifest.tsv final | sed -n '4p'
echo "== pass 1: every unused import dropped (upper bound) -> classify -> c01 strip list"
$PY work/effective.py $SRC imports.json manifest.tsv step1_splits.json work/out_pass1 >/dev/null
$PY work/external_imports.py ../wt-replan work/external_imports.json >/dev/null
$PY work/classify_drops.py work/out_pass1 work/drops.tsv | sed -n '1,3p'
$PY work/make_strip_list.py work/drops.tsv step1_reexports.tsv | head -1
echo "== pass 2: only listed lines dropped"
$PY work/effective.py $SRC imports.json manifest.tsv step1_splits.json "$OUT" step1_reexports.tsv
for s in step1 final; do
  $PY check_rules.py "$OUT/eff_imports_$s.json" "$OUT/eff_manifest_$s.tsv" $s --list "$OUT/v_$s.tsv" | sed -n '3,40p'
done
echo "== pure-vision walk (no exceptions) -> frontier"
$PY work/vision_reach.py "$OUT/eff_imports_step1.json" "$OUT/eff_manifest_step1.tsv" step1 \
  | sed -n '/frontier/,$p' > "$OUT/vision_frontier.txt"
head -1 "$OUT/vision_frontier.txt"
$PY work/gen_exceptions.py "$OUT" work/round1/step1_exceptions.tsv step1_exceptions.tsv
echo "== pure-vision walk with the named exceptions"
$PY work/vision_reach.py "$OUT/eff_imports_step1.json" "$OUT/eff_manifest_step1.tsv" step1 step1_exceptions.tsv \
  | grep -E "^stage|^frontier|^stale"
cp "$OUT/v_final.tsv" final_violations.tsv
