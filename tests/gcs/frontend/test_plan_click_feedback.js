import assert from 'node:assert/strict';
import test from 'node:test';

import {
  cartographicToMapPoint,
  planVertexBillboardPlacement,
} from '../../../src/gcs/frontend/src/components/map/utils/planVertexPlacement.js';

const Cesium = {
  Math: {
    toDegrees: (value) => value * 10,
  },
  Cartesian3: {
    fromDegrees: (...args) => args,
  },
  HeightReference: {
    NONE: 'none',
    CLAMP_TO_GROUND: 'clamp',
  },
};

test('picked terrain height is preserved in the map point', () => {
  assert.deepEqual(
    cartographicToMapPoint(Cesium, { latitude: 1, longitude: 2, height: 347.5 }),
    { lat: 10, lon: 20, height: 347.5 },
  );
});

test('freshly picked vertices render immediately at the known terrain height', () => {
  const placement = planVertexBillboardPlacement(Cesium, {
    lat: 10,
    lon: 20,
    height: 347.5,
  });

  assert.deepEqual(placement, {
    position: [20, 10, 347.5],
    heightReference: 'none',
  });
});

test('loaded vertices without a sampled height retain terrain clamping', () => {
  const placement = planVertexBillboardPlacement(Cesium, { lat: 10, lon: 20 });

  assert.deepEqual(placement, {
    position: [20, 10],
    heightReference: 'clamp',
  });
});
