const { latLonToLocal, localToLatLon, pointInPolygon } = require('./geo');

function magnitude(vector) {
  return Math.hypot(vector.x, vector.y);
}

function normalize(vector) {
  const length = magnitude(vector) || 1;
  return { x: vector.x / length, y: vector.y / length };
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

function resolveCoverageFrame(localPolygon) {
  if (localPolygon.length < 3) {
    return {
      axisU: { x: 1, y: 0 },
      axisV: { x: 0, y: 1 },
      polygon: localPolygon.map((point) => ({ x: point.x, y: point.y })),
      minX: 0,
      minY: 0,
      maxX: 0,
      maxY: 0,
    };
  }

  const rawU = {
    x: localPolygon[1].x - localPolygon[0].x,
    y: localPolygon[1].y - localPolygon[0].y,
  };
  const axisU = normalize(magnitude(rawU) > 0 ? rawU : {
    x: localPolygon[2].x - localPolygon[0].x,
    y: localPolygon[2].y - localPolygon[0].y,
  });
  let axisV = { x: -axisU.y, y: axisU.x };

  const sideVector = {
    x: (localPolygon[localPolygon.length - 1]?.x ?? 0) - localPolygon[0].x,
    y: (localPolygon[localPolygon.length - 1]?.y ?? 0) - localPolygon[0].y,
  };
  if (dot(sideVector, axisV) < 0) {
    axisV = { x: -axisV.x, y: -axisV.y };
  }

  const projectedPolygon = localPolygon.map((point) => ({
    x: dot(point, axisU),
    y: dot(point, axisV),
  }));

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const point of projectedPolygon) {
    minX = Math.min(minX, point.x);
    minY = Math.min(minY, point.y);
    maxX = Math.max(maxX, point.x);
    maxY = Math.max(maxY, point.y);
  }

  return {
    axisU,
    axisV,
    polygon: projectedPolygon,
    minX,
    minY,
    maxX,
    maxY,
  };
}

function localToFrame(localPoint, map) {
  return {
    x: dot(localPoint, map.axisU),
    y: dot(localPoint, map.axisV),
  };
}

function frameToLocal(framePoint, map) {
  return {
    x: framePoint.x * map.axisU.x + framePoint.y * map.axisV.x,
    y: framePoint.x * map.axisU.y + framePoint.y * map.axisV.y,
  };
}

function buildCoverageMap(boundaryLatLon, cellSizeM = 2.0) {
  const origin = {
    lat: boundaryLatLon[0].lat,
    lon: boundaryLatLon[0].lon,
  };

  const localPolygon = boundaryLatLon.map((point) => latLonToLocal(point, origin));
  const frame = resolveCoverageFrame(localPolygon);

  const width = Math.max(1, Math.ceil((frame.maxX - frame.minX) / cellSizeM));
  const height = Math.max(1, Math.ceil((frame.maxY - frame.minY) / cellSizeM));

  const cells = Array.from({ length: height }, () =>
    Array.from({ length: width }, () => ({
      inside: false,
      covered: false,
      hits: 0,
      lastSeenMs: 0,
    }))
  );

  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const center = {
        x: frame.minX + (col + 0.5) * cellSizeM,
        y: frame.minY + (row + 0.5) * cellSizeM,
      };
      cells[row][col].inside = pointInPolygon(center, frame.polygon);
    }
  }

  return {
    origin,
    polygon: localPolygon,
    axisU: frame.axisU,
    axisV: frame.axisV,
    projectedPolygon: frame.polygon,
    minX: frame.minX,
    minY: frame.minY,
    maxX: frame.maxX,
    maxY: frame.maxY,
    width,
    height,
    cellSizeM,
    cells,
  };
}

function worldToGrid(map, pointLatLon) {
  const local = latLonToLocal(pointLatLon, map.origin);
  const framePoint = localToFrame(local, map);
  const col = Math.floor((framePoint.x - map.minX) / map.cellSizeM);
  const row = Math.floor((framePoint.y - map.minY) / map.cellSizeM);
  return { row, col };
}

function gridToWorld(map, row, col) {
  const framePoint = {
    x: map.minX + (col + 0.5) * map.cellSizeM,
    y: map.minY + (row + 0.5) * map.cellSizeM,
  };
  const local = frameToLocal(framePoint, map);
  return localToLatLon(local, map.origin);
}

function gridCellPolygon(map, row, col) {
  const x0 = map.minX + col * map.cellSizeM;
  const y0 = map.minY + row * map.cellSizeM;
  const x1 = x0 + map.cellSizeM;
  const y1 = y0 + map.cellSizeM;

  return [
    localToLatLon(frameToLocal({ x: x0, y: y0 }, map), map.origin),
    localToLatLon(frameToLocal({ x: x1, y: y0 }, map), map.origin),
    localToLatLon(frameToLocal({ x: x1, y: y1 }, map), map.origin),
    localToLatLon(frameToLocal({ x: x0, y: y1 }, map), map.origin),
  ];
}

function withinGrid(map, row, col) {
  return row >= 0 && row < map.height && col >= 0 && col < map.width;
}

function markCoverage(map, pointLatLon, radiusM = 1.5, timestampMs = Date.now()) {
  const center = worldToGrid(map, pointLatLon);
  const radiusCells = Math.max(0, Math.ceil(radiusM / map.cellSizeM));

  for (let dr = -radiusCells; dr <= radiusCells; dr++) {
    for (let dc = -radiusCells; dc <= radiusCells; dc++) {
      const row = center.row + dr;
      const col = center.col + dc;
      if (!withinGrid(map, row, col)) continue;

      const cell = map.cells[row][col];
      if (!cell.inside) continue;

      const distance = Math.hypot(dr * map.cellSizeM, dc * map.cellSizeM);
      if (distance <= radiusM) {
        cell.covered = true;
        cell.hits += 1;
        cell.lastSeenMs = timestampMs;
      }
    }
  }
}

function coverageStats(map) {
  let insideCount = 0;
  let coveredCount = 0;

  for (const row of map.cells) {
    for (const cell of row) {
      if (!cell.inside) continue;
      insideCount += 1;
      if (cell.covered) coveredCount += 1;
    }
  }

  const coveragePercent = insideCount ? (coveredCount / insideCount) * 100 : 0;

  return {
    insideCount,
    coveredCount,
    coveragePercent,
    coveredPct: coveragePercent,
  };
}

module.exports = {
  buildCoverageMap,
  resolveCoverageFrame,
  worldToGrid,
  gridToWorld,
  gridCellPolygon,
  withinGrid,
  markCoverage,
  coverageStats,
};
