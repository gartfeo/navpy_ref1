// No app code calls the browser's blocking confirm(): it freezes the page
// (telemetry, HUD, E-STOP) and, in the handheld app's WebView, brings back
// Android's navigation bar (todo 233). Use hooks/useConfirm instead.
//
// Parsed, not grepped: a bare confirm() is native only where no `confirm`
// binding is in scope at that call, so a top-level helper calling the native
// dialog is caught even beside a hook that has its own local confirm.
// @babel/parser and @babel/traverse come with @vitejs/plugin-react.
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join, relative } from 'node:path';
import { parse } from '@babel/parser';
import traverseModule from '@babel/traverse';
import { describe, expect, it } from 'vitest';

const traverse = traverseModule.default ?? traverseModule;

// vitest runs from the frontend folder (package.json).
const SRC = join(process.cwd(), 'src');
const SKIP_DIRS = new Set(['test', 'vendor']);
const GLOBALS = new Set(['window', 'globalThis', 'self']);

function sourceFiles(dir) {
  return readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    if (statSync(path).isDirectory()) return SKIP_DIRS.has(name) ? [] : sourceFiles(path);
    return /\.(jsx?|tsx?)$/.test(name) && !/\.test\./.test(name) ? [path] : [];
  });
}

// Parsing and scope-tracking every source file costs over a second of CPU.
// An identifier or string the AST reads as `confirm` is either the word
// `confirm` in the text or written with an escape (confirm, '\x63onfirm',
// a line continuation), and every escape needs a backslash, so a file with
// neither cannot reach the native dialog and is not parsed. That skips about
// three quarters of the files and halves the scan.
function mayNameConfirm(code) {
  return /\bconfirm\b/.test(code) || code.includes('\\');
}

// The scan still parses the large files, about 0.6-1.2 s alone. A full
// `npm test` runs one worker per core, which stretched the old full scan from
// 1.6 s alone to 5.2-8.2 s (2026-11-09), past vitest's 5 s unit-test default.
// This budget is for a whole-tree scan under that load; a hang still fails.
const TREE_SCAN_TIMEOUT_MS = 30_000;

/** Line numbers where `code` reaches the browser's confirm. */
export function nativeConfirmLines(code) {
  if (!mayNameConfirm(code)) return [];
  const ast = parse(code, { sourceType: 'module', plugins: ['jsx'] });
  const lines = [];
  traverse(ast, {
    // window.confirm / globalThis.confirm / self.confirm, called or passed on.
    MemberExpression(path) {
      const { object, property, computed } = path.node;
      const name = computed ? property.value : property.name;
      if (object.type === 'Identifier' && GLOBALS.has(object.name) && name === 'confirm') {
        lines.push(path.node.loc.start.line);
      }
    },
    // A bare confirm(...) with no `confirm` binding in scope is the global one.
    CallExpression(path) {
      const { callee } = path.node;
      if (callee.type === 'Identifier' && callee.name === 'confirm' && !path.scope.hasBinding('confirm')) {
        lines.push(path.node.loc.start.line);
      }
    },
  });
  return lines;
}

describe('no native confirm()', () => {
  it('app code asks through the in-page confirm', () => {
    const offenders = sourceFiles(SRC).flatMap((file) => nativeConfirmLines(readFileSync(file, 'utf8'))
      .map((n) => `${relative(SRC, file)}:${n}`));
    expect(offenders).toEqual([]);
  }, TREE_SCAN_TIMEOUT_MS);

  it('a top-level helper calling confirm is caught beside a hook with its own confirm', () => {
    const code = [
      "import useConfirm from './useConfirm';",
      'function sure(msg) { return confirm(msg); }',
      'export default function useX() {',
      '  const confirm = useConfirm();',
      "  return async () => (await confirm('a?')) && sure('b?');",
      '}',
    ].join('\n');
    expect(nativeConfirmLines(code)).toEqual([2]);
  });

  it('catches window/globalThis/self.confirm, called or passed', () => {
    const code = [
      "window.confirm('a');",
      "globalThis['confirm']('b');",
      'const f = self.confirm;',
      'run({ confirm: window.confirm });',
    ].join('\n');
    expect(nativeConfirmLines(code)).toEqual([1, 2, 3, 4]);
  });

  it('a parameter or local named confirm, or a method, is not native', () => {
    const code = [
      'export async function check({ confirm }) { return confirm("x"); }',
      'function g(confirm) { return confirm("y"); }',
      'const h = () => { const confirm = ask; return confirm("z"); };',
      'api.confirm("w"); const el = <Modal onConfirm={() => done()} />;',
    ].join('\n');
    expect(nativeConfirmLines(code)).toEqual([]);
  });

  it('confirm written with escapes is still parsed and caught', () => {
    const code = [
      "window['\\x63onfirm']('a');",
      '\\u0063onfirm("b");',
      "self['conf\\",
      "irm']('c');",
    ].join('\n');
    expect(nativeConfirmLines(code)).toEqual([1, 2, 3]);
  });
});
