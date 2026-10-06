/**
 * Build-time version for the web-app manifest URL.
 *
 * On the Android handheld, reinstalling the home-screen app from the same
 * manifest URL kept the old launcher icon, even after the old shortcut was
 * removed and the server returned the new manifest; Chrome did not fetch the
 * new icons at all. A new manifest URL made Chrome fetch them. The build
 * therefore links the manifest with a version that changes whenever the
 * manifest or one of its icons changes.
 */
import { createHash } from 'node:crypto'
import { readFileSync } from 'node:fs'
import path from 'node:path'

const MANIFEST_LINK = /(<link rel="manifest" href="\/manifest\.json)(")/

export function manifestVersion(publicDir) {
  const manifest = readFileSync(path.join(publicDir, 'manifest.json'))
  const hash = createHash('sha256').update(manifest)
  for (const icon of JSON.parse(manifest).icons ?? []) {
    hash.update(readFileSync(path.join(publicDir, icon.src)))
  }
  return hash.digest('hex').slice(0, 10)
}

export function versionManifestLink(html, version) {
  if (!MANIFEST_LINK.test(html)) {
    throw new Error('index.html has no <link rel="manifest" href="/manifest.json">')
  }
  return html.replace(MANIFEST_LINK, `$1?v=${version}$2`)
}

export function manifestVersionPlugin(publicDir) {
  return {
    name: 'manifest-version',
    apply: 'build',
    transformIndexHtml: (html) => versionManifestLink(html, manifestVersion(publicDir)),
  }
}
