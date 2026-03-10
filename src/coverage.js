const { latLonToLocal, localToLatLon, pointInPolygon } = require('./geo');

function buildCoverageMap(boundaryLatLon, cellSizeM = 2.0) {
  const origin = {
    lat: boundaryLatLon[0].lat,
    lon: boundaryLatLon[0].lon,
  };

  const polygon = boundaryLatLon.map((point) => latLonToLocal(point, origin));

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (const point of polygon) {
    minX = Math.min(minX, point.x);
    minY = Math.min(minY, point.y);
    maxX = Math.max(maxX, point.x);
    maxY = Math.max(maxY, point.y);
  }

  const width = Math.max(1, Math.ceil((maxX - minX) / cellSizeM));
  const height = Math.max(1, Math.ceil((maxY - minY) / cellSizeM));

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
        x: minX + (col + 0.5) * cellSizeM,
        y: minY + (row + 0.5) * cellSizeM,
      };
      cells[row][col].inside = pointInPolygon(center, polygon);
    }
  }

  return {
    origin,
    polygon,
    minX,
    minY,
    maxX,
    maxY,
    width,
    height,
    cellSizeM,
    cells,
  };
}

function worldToGrid(map, pointLatLon) {
  const local = latLonToLocal(pointLatLon, map.origin);
  const col = Math.floor((local.x - map.minX) / map.cellSizeM);
  const row = Math.floor((local.y - map.minY) / map.cellSizeM);
  return { row, col };
}

function gridToWorld(map, row, col) {
  const local = {
    x: map.minX + (col + 0.5) * map.cellSizeM,
    y: map.minY + (row + 0.5) * map.cellSizeM,
  };
  return localToLatLon(local, map.origin);
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

  return {
    insideCount,
    coveredCount,
    coveragePercent: insideCount ? (coveredCount / insideCount) * 100 : 0,
  };
}

module.exports = {
  buildCoverageMap,
  worldToGrid,
  gridToWorld,
  withinGrid,
  markCoverage,
  coverageStats,
};
