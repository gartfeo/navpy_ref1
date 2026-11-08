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

/** Line numbers where `code` reaches the browser's confirm. */
export function nativeConfirmLines(code) {
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
  });

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
});
