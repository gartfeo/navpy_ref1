import { beforeEach, expect, it, vi } from 'vitest';
import { handleClick } from './handleClick';
import { pickEntity } from '../../utils/entityPicking';
import { flyToClosestApproach } from '../../utils/closestApproachView';

vi.mock('../../utils/entityPicking', () => ({ pickEntity: vi.fn(), pickCartographic: vi.fn(() => ({ lat: 1, lon: 2 })) }));
vi.mock('../../utils/closestApproachView', () => ({ flyToClosestApproach: vi.fn() }));

const view = { lon: 44, lat: 40, alt: 41, headingDeg: 91 };
const click = { position: { x: 1, y: 2 } };

beforeEach(() => {
  pickEntity.mockReset();
  flyToClosestApproach.mockReset();
});

it('flies to a closest-approach mark in monitor mode', () => {
  pickEntity.mockReturnValue({ type: 'closestApproach', key: 'b:1:1', view });
  handleClick('Cesium', 'viewer', click, { current: { editable: false } });
  expect(flyToClosestApproach).toHaveBeenCalledExactlyOnceWith('Cesium', 'viewer', view);
});

it('does nothing for another entity outside drawing', () => {
  pickEntity.mockReturnValue({ type: 'polygon' });
  handleClick('Cesium', 'viewer', click, { current: { editable: true, isDrawing: false } });
  expect(flyToClosestApproach).not.toHaveBeenCalled();
});

it('keeps adding vertices while drawing', () => {
  pickEntity.mockReturnValue({ type: 'closestApproach', key: 'b:1:1', view });
  const onMapClick = vi.fn();
  handleClick('Cesium', 'viewer', click, { current: { editable: true, isDrawing: true, onMapClick } });
  expect(onMapClick).toHaveBeenCalledExactlyOnceWith({ lat: 1, lon: 2 });
  expect(flyToClosestApproach).not.toHaveBeenCalled();
});
