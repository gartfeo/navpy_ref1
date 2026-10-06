/**
 * Node.js tests for the build-time manifest URL version.
 *
 * Run via: node tests/gcs/frontend/test_manifest_version.mjs
 */
import assert from 'node:assert';
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  manifestVersion,
  manifestVersionPlugin,
  versionManifestLink,
} from '../../../src/gcs/frontend/manifestVersion.js';

const REAL_PUBLIC = fileURLToPath(new URL('../../../src/gcs/frontend/public', import.meta.url));

// The link gets the version; the rest of the page is untouched.
{
  const html = '<head><link rel="icon" href="/a.svg"><link rel="manifest" href="/manifest.json"></head>';
  assert.strictEqual(
    versionManifestLink(html, 'abc123'),
    '<head><link rel="icon" href="/a.svg"><link rel="manifest" href="/manifest.json?v=abc123"></head>',
  );
}

// A page without the expected link fails the build instead of shipping unversioned.
assert.throws(() => versionManifestLink('<head></head>', 'abc123'), /manifest/);

// The version follows the manifest and every icon it lists.
{
  const dir = mkdtempSync(path.join(tmpdir(), 'manifest-version-'));
  try {
    const manifest = (name) => JSON.stringify({ name, icons: [{ src: '/icon.png' }, { src: '/icon-maskable.png' }] });
    writeFileSync(path.join(dir, 'manifest.json'), manifest('GCS'));
    writeFileSync(path.join(dir, 'icon.png'), 'any');
    writeFileSync(path.join(dir, 'icon-maskable.png'), 'maskable-1');
    const first = manifestVersion(dir);
    assert.match(first, /^[0-9a-f]{10}$/);
    assert.strictEqual(manifestVersion(dir), first, 'same files, same version');

    writeFileSync(path.join(dir, 'icon-maskable.png'), 'maskable-2');
    const iconChanged = manifestVersion(dir);
    assert.notStrictEqual(iconChanged, first, 'a changed icon changes the version');

    writeFileSync(path.join(dir, 'manifest.json'), manifest('AAS GCS'));
    assert.notStrictEqual(manifestVersion(dir), iconChanged, 'a changed manifest changes the version');

    // A listed icon that is missing fails the build.
    writeFileSync(path.join(dir, 'manifest.json'), JSON.stringify({ icons: [{ src: '/gone.png' }] }));
    assert.throws(() => manifestVersion(dir), /ENOENT/);
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

// The shipped manifest's icons all exist, so the real build can version it.
assert.match(manifestVersion(REAL_PUBLIC), /^[0-9a-f]{10}$/);

// Build only: the dev server keeps the plain link.
{
  const plugin = manifestVersionPlugin(REAL_PUBLIC);
  assert.strictEqual(plugin.apply, 'build');
  const out = plugin.transformIndexHtml('<link rel="manifest" href="/manifest.json">');
  assert.strictEqual(out, `<link rel="manifest" href="/manifest.json?v=${manifestVersion(REAL_PUBLIC)}">`);
}

console.log('manifest version: all tests passed');
