// No app code calls the browser's blocking confirm(): it freezes the page
// (telemetry, HUD, E-STOP) and, in the handheld app's WebView, brings back
// Android's navigation bar (todo 233). Use hooks/useConfirm instead.
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join, relative } from 'node:path';
import { describe, expect, it } from 'vitest';

// vitest runs from the frontend folder (package.json).
const SRC = join(process.cwd(), 'src');
const SKIP_DIRS = new Set(['test', 'vendor']);

function sourceFiles(dir) {
  return readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    if (statSync(path).isDirectory()) return SKIP_DIRS.has(name) ? [] : sourceFiles(path);
    return /\.(jsx?|tsx?)$/.test(name) && !/\.test\./.test(name) ? [path] : [];
  });
}

// window.confirm(, globalThis.confirm(, or a bare confirm( call that is not
// a method (x.confirm) or a longer name (onConfirm, isConfirm).
const NATIVE_CONFIRM = /\b(?:window|globalThis|self)\.confirm\b|(?<![\w.$])confirm\s*\(/;
// A local named `confirm` (from useConfirm, or a parameter) makes a bare call fine.
const LOCAL_CONFIRM = /\b(?:const|let|var)\s+confirm\b|[({,]\s*confirm\s*[,})=]|useConfirm\(\)/;

describe('no native confirm()', () => {
  it('app code asks through the in-page confirm', () => {
    const offenders = sourceFiles(SRC).flatMap((file) => {
      const text = readFileSync(file, 'utf8');
      const lines = text.split('\n')
        .map((line, i) => ({ line: line.replace(/\/\/.*$/, ''), n: i + 1 }))
        .filter(({ line }) => !/^\s*(?:\/\*|\*)/.test(line) && NATIVE_CONFIRM.test(line));
      const native = lines.filter(({ line }) => /\b(?:window|globalThis|self)\.confirm\b/.test(line)
        || !LOCAL_CONFIRM.test(text));
      return native.map(({ n }) => `${relative(SRC, file)}:${n}`);
    });
    expect(offenders).toEqual([]);
  });
});
