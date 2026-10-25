import { expect, it } from 'vitest';
import { pickEntity } from './entityPicking';

const Cesium = { defined: (value) => value !== undefined && value !== null };
const property = (value) => ({ getValue: () => value });

it('picks a closest-approach mark with its camera view and its entity', () => {
  const view = { lon: 44, lat: 40, alt: 41, headingDeg: null };
  const entity = { properties: {
    closestApproachKey: property('b:1:1'), closestApproachView: property(view),
  } };
  const viewer = { scene: { drillPick: () => [{ id: entity }] } };
  const hit = pickEntity(Cesium, viewer, {});
  expect(hit).toEqual({ type: 'closestApproach', key: 'b:1:1', view, entity });
  expect(hit.entity).toBe(entity);
});
