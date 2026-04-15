const test = require('node:test');
const assert = require('node:assert/strict');

const { buildCoverageMap, gridCellPolygon } = require('../src/coverage');
const { latLonToLocal } = require('../src/geo');
const { findCoveragePath, findPath, fitWaypointsToLimit } = require('../src/pathing');

function createRectBoundary(origin, widthM, heightM) {
  const latPerM = 1 / 111320;
  const lonPerM = 1 / (111320 * Math.cos((origin.lat * Math.PI) / 180));

  return [
    { lat: origin.lat, lon: origin.lon },
    { lat: origin.lat, lon: origin.lon + widthM * lonPerM },
    { lat: origin.lat + heightM * latPerM, lon: origin.lon + widthM * lonPerM },
    { lat: origin.lat + heightM * latPerM, lon: origin.lon },
  ];
}

function createRotatedRectBoundary(origin, widthM, heightM, angleDeg) {
  const angle = (angleDeg * Math.PI) / 180;
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const corners = [
    { x: 0, y: 0 },
    { x: widthM, y: 0 },
    { x: widthM, y: heightM },
    { x: 0, y: heightM },
  ].map((point) => ({
    x: point.x * cos - point.y * sin,
    y: point.x * sin + point.y * cos,
  }));

  const latPerM = 1 / 111320;
  const lonPerM = 1 / (111320 * Math.cos((origin.lat * Math.PI) / 180));
  return corners.map((point) => ({
    lat: origin.lat + point.y * latPerM,
    lon: origin.lon + point.x * lonPerM,
  }));
}

test('coverage path orients from the selected start corner toward the opposite corner', () => {
  const boundary = createRectBoundary({ lat: 41.0, lon: -81.5 }, 6, 8);
  const map = buildCoverageMap(boundary, 1);

  const start = boundary[0];
  const goal = boundary[2];
  const result = findCoveragePath(map, { swathWidthM: 1, startLatLon: start, goalLatLon: goal });

  assert.equal(result.ok, true);
  assert.ok(result.points.length > 4);
  assert.equal(result.points[0].headingDeg, 90);
  assert.ok(result.meta.pointCount >= result.points.length);
});

test('coverage path can reverse orientation when start and goal corners are swapped', () => {
  const boundary = createRectBoundary({ lat: 41.0, lon: -81.5 }, 6, 8);
  const map = buildCoverageMap(boundary, 1);

  const start = boundary[2];
  const goal = boundary[0];
  const result = findCoveragePath(map, { swathWidthM: 1, startLatLon: start, goalLatLon: goal });

  assert.equal(result.ok, true);
  assert.ok(result.points.length > 4);
  assert.equal(result.points[0].headingDeg, 270);
});

test('goal path snaps from outside the area to the nearest interior cell', () => {
  const boundary = createRectBoundary({ lat: 41.0, lon: -81.5 }, 6, 6);
  const map = buildCoverageMap(boundary, 1);

  const result = findPath(
    map,
    { lat: boundary[0].lat - 0.00003, lon: boundary[0].lon - 0.00003 },
    { lat: boundary[2].lat + 0.00003, lon: boundary[2].lon + 0.00003 },
  );

  assert.equal(result.ok, true);
  assert.ok(result.points.length >= 2);
  assert.ok(result.points[0].headingDeg !== null);
});

test('coverage path succeeds for rotated rectangles and keeps coverage cells aligned', () => {
  const boundary = createRotatedRectBoundary({ lat: 41.0, lon: -81.5 }, 10, 6, 32);
  const map = buildCoverageMap(boundary, 1);
  const result = findCoveragePath(map, { swathWidthM: 1, startLatLon: boundary[0], goalLatLon: boundary[2] });

  assert.equal(result.ok, true);
  assert.ok(result.points.length > 6);
  assert.ok(map.width >= 5);
  assert.ok(map.height >= 5);
  assert.ok(map.projectedPolygon.every((point) => Number.isFinite(point.x) && Number.isFinite(point.y)));

   const firstInsideRow = map.cells.findIndex((row) => row.some((cell) => cell.inside));
   assert.ok(firstInsideRow >= 0);
   const firstInsideCol = map.cells[firstInsideRow].findIndex((cell) => cell.inside);
   assert.ok(firstInsideCol >= 0);

   const cell = gridCellPolygon(map, firstInsideRow, firstInsideCol);
   const a = latLonToLocal(cell[0], map.origin);
   const b = latLonToLocal(cell[1], map.origin);
   const edgeAngleDeg = (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI;
   const normalizedAngle = Math.abs(((edgeAngleDeg % 90) + 90) % 90);

   assert.ok(normalizedAngle > 10 && normalizedAngle < 80, `expected rotated cell edge, got ${edgeAngleDeg.toFixed(2)}°`);
   assert.ok(result.points.some((point) => point.headingDeg !== 0 && point.headingDeg !== 90 && point.headingDeg !== 180 && point.headingDeg !== 270));
});

test('fitWaypointsToLimit preserves the full route span when reducing oversized paths', () => {
  const points = Array.from({ length: 360 }, (_value, index) => ({
    lat: 41 + (index * 0.000001),
    lon: -81.5 + ((index % 15) * 0.000001),
    salt: index % 100,
    brine: 100 - (index % 100),
  }));

  const fitted = fitWaypointsToLimit(points, 120);

  assert.equal(fitted.length, 120);
  assert.deepEqual(fitted[0], points[0]);
  assert.deepEqual(fitted[fitted.length - 1], points[points.length - 1]);
  assert.ok(fitted[60].lat > points[150].lat, 'expected sampled path to retain later mission points');
});
