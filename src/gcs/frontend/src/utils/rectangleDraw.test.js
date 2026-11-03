import { describe, expect, it } from 'vitest';
import { MIN_RECT_SIDE_M, rectangleCorners, rectanglePreview, sideLength } from './rectangleDraw';
import { METERS_PER_DEG, offsetLatLon } from './geo';

// North/east metres of p from origin o, at o's latitude.
const local = (o, p) => ({
  n: (p.lat - o.lat) * METERS_PER_DEG,
  e: (p.lon - o.lon) * METERS_PER_DEG * Math.cos(o.lat * Math.PI / 180),
});
const headingDeg = (o, p) => {
  const { n, e } = local(o, p);
  return ((Math.atan2(e, n) * 180) / Math.PI + 360) % 360;
};
const along = (o, deg, m) => offsetLatLon(o.lat, o.lon, m * Math.cos(deg * Math.PI / 180), m * Math.sin(deg * Math.PI / 180));

describe('rectangleCorners', () => {
  // The owner's case: a 1.26 x 0.77 km box along a road at about 74 deg.
  const a = { lat: 40.18, lon: 44.51 };
  const b = along(a, 74, 1260);
  // Third click: 770 m to the right of the side, and slid 300 m along it;
  // only its perpendicular distance counts.
  const c = along(along(a, 74, 300), 74 + 90, 770);

  it('keeps the first side and makes a right-angled 1260 x 770 m box', () => {
    const [p0, p1, p2, p3] = rectangleCorners(a, b, c);
    expect(p0).toEqual(a);
    expect(p1).toEqual(b);
    expect(sideLength(p0, p1)).toBeCloseTo(1260, 3);
    expect(sideLength(p1, p2)).toBeCloseTo(770, 0);
    expect(sideLength(p2, p3)).toBeCloseTo(1260, 0);
    expect(sideLength(p3, p0)).toBeCloseTo(770, 0);
    expect(headingDeg(p0, p1)).toBeCloseTo(74, 6);
    expect(headingDeg(p0, p3)).toBeCloseTo(164, 3);
    expect(headingDeg(p1, p2)).toBeCloseTo(164, 3);
  });

  it('puts the width on the side of the third click', () => {
    const left = along(a, 74 - 90, 770);
    const [, , , p3] = rectangleCorners(a, b, left);
    expect(headingDeg(a, p3)).toBeCloseTo(344, 3);
    expect(sideLength(a, p3)).toBeCloseTo(770, 0);
  });

  it('refuses a side or width shorter than the minimum', () => {
    expect(rectangleCorners(a, a, c)).toBeNull();
    expect(rectangleCorners(a, along(a, 74, MIN_RECT_SIDE_M / 2), c)).toBeNull();
    // A third click on the first side's line gives no width.
    expect(rectangleCorners(a, b, along(a, 74, 600))).toBeNull();
  });
});

describe('rectanglePreview', () => {
  const a = { lat: 40.18, lon: 44.51 };
  const b = along(a, 74, 1260);
  const cursor = along(b, 164, 500);

  it('shows the side to the pointer after one click', () => {
    expect(rectanglePreview([a], cursor)).toEqual([a, cursor]);
  });

  it('shows the full rectangle after two clicks', () => {
    expect(rectanglePreview([a, b], cursor)).toEqual(rectangleCorners(a, b, cursor));
  });

  it('shows just the side while the pointer is on its line', () => {
    expect(rectanglePreview([a, b], along(a, 74, 2000))).toEqual([a, b]);
  });

  it('shows nothing without a draft or pointer', () => {
    expect(rectanglePreview([], cursor)).toEqual([]);
    expect(rectanglePreview([a], null)).toEqual([]);
    expect(rectanglePreview([a, b, cursor], cursor)).toEqual([]);
  });
});
