// @vitest-environment node
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';

/**
 * The UAV model (public/Peron15.glb) is drawn once per UAV every frame, so
 * its draw calls and triangles are paid per UAV per frame. The CAD export
 * had 432 meshes (864 draw calls per UAV with the silhouette) and 323k
 * triangles, about two thirds of the map's render time (todo 287). It was
 * merged and simplified with gltf-transform 4.5.1:
 *   gltf-transform optimize Peron15.glb out.glb --compress false
 *     --texture-compress false --instance false
 *     --simplify-ratio 0.1 --simplify-error 0.002
 * Re-run that on any new export.
 */
const MAX_PRIMITIVES = 8;
const MAX_TRIANGLES = 50000;

function readGltfJson(path) {
  const bytes = readFileSync(path);
  // GLB: 12-byte header, then the JSON chunk (length, type, data).
  const jsonLength = bytes.readUInt32LE(12);
  return JSON.parse(bytes.subarray(20, 20 + jsonLength).toString('utf8'));
}

describe('UAV model asset', () => {
  const gltf = readGltfJson(fileURLToPath(new URL('../../../public/Peron15.glb', import.meta.url)));
  const primitives = gltf.meshes.flatMap((m) => m.primitives);

  it('draws in a few calls per UAV', () => {
    expect(primitives.length).toBeLessThanOrEqual(MAX_PRIMITIVES);
  });

  it('keeps the triangle count small', () => {
    const triangles = primitives.reduce((n, p) => n + gltf.accessors[p.indices].count / 3, 0);
    expect(triangles).toBeLessThanOrEqual(MAX_TRIANGLES);
  });

  it('needs no decoder extension', () => {
    expect(gltf.extensionsRequired ?? []).toEqual([]);
  });
});
