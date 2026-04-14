const { worldToGrid, gridToWorld, withinGrid } = require('./coverage');

function key(row, col) {
  return `${row}:${col}`;
}

function heuristic(a, b) {
  return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
}

function neighbors(node) {
  return [
    { row: node.row - 1, col: node.col },
    { row: node.row + 1, col: node.col },
    { row: node.row, col: node.col - 1 },
    { row: node.row, col: node.col + 1 },
  ];
}

function isWalkable(map, row, col) {
  if (!withinGrid(map, row, col)) return false;
  return map.cells[row][col].inside;
}

function nearestWalkable(map, fromRow, fromCol) {
  if (isWalkable(map, fromRow, fromCol)) {
    return { row: fromRow, col: fromCol };
  }

  const maxRadius = Math.max(map.height, map.width);
  for (let radius = 1; radius <= maxRadius; radius++) {
    const minRow = fromRow - radius;
    const maxRow = fromRow + radius;
    const minCol = fromCol - radius;
    const maxCol = fromCol + radius;

    for (let row = minRow; row <= maxRow; row++) {
      if (isWalkable(map, row, minCol)) return { row, col: minCol };
      if (isWalkable(map, row, maxCol)) return { row, col: maxCol };
    }

    for (let col = minCol + 1; col <= maxCol - 1; col++) {
      if (isWalkable(map, minRow, col)) return { row: minRow, col };
      if (isWalkable(map, maxRow, col)) return { row: maxRow, col };
    }
  }

  return null;
}

function reconstructPath(cameFrom, current) {
  const result = [current];
  let cursor = current;
  while (cameFrom.has(key(cursor.row, cursor.col))) {
    cursor = cameFrom.get(key(cursor.row, cursor.col));
    result.push(cursor);
  }
  return result.reverse();
}

function findGridPath(map, start, goal) {
  const open = [start];
  const openSet = new Set([key(start.row, start.col)]);
  const cameFrom = new Map();
  const gScore = new Map([[key(start.row, start.col), 0]]);
  const fScore = new Map([[key(start.row, start.col), heuristic(start, goal)]]);

  while (open.length) {
    open.sort((a, b) => (fScore.get(key(a.row, a.col)) ?? Infinity) - (fScore.get(key(b.row, b.col)) ?? Infinity));
    const current = open.shift();
    openSet.delete(key(current.row, current.col));

    if (current.row === goal.row && current.col === goal.col) {
      return reconstructPath(cameFrom, current);
    }

    for (const next of neighbors(current)) {
      if (!isWalkable(map, next.row, next.col)) continue;

      const tentative = (gScore.get(key(current.row, current.col)) ?? Infinity) + 1;
      const nextKey = key(next.row, next.col);

      if (tentative < (gScore.get(nextKey) ?? Infinity)) {
        cameFrom.set(nextKey, current);
        gScore.set(nextKey, tentative);
        fScore.set(nextKey, tentative + heuristic(next, goal));

        if (!openSet.has(nextKey)) {
          open.push(next);
          openSet.add(nextKey);
        }
      }
    }
  }

  return null;
}

function splitContiguous(cols) {
  if (!cols.length) return [];

  const runs = [];
  let start = cols[0];
  let prev = cols[0];

  for (let i = 1; i < cols.length; i++) {
    const value = cols[i];
    if (value === prev + 1) {
      prev = value;
      continue;
    }
    runs.push({ start, end: prev });
    start = value;
    prev = value;
  }

  runs.push({ start, end: prev });
  return runs;
}

function headingDeg(from, to) {
  const avgLatRad = ((from.lat + to.lat) * Math.PI) / 360;
  const dNorth = to.lat - from.lat;
  const dEast = (to.lon - from.lon) * Math.cos(avgLatRad);
  const angle = (Math.atan2(dEast, dNorth) * 180) / Math.PI;
  return (angle + 360) % 360;
}

function haversineDistanceMeters(a, b) {
  const earthRadiusM = 6371000;
  const lat1 = a.lat * (Math.PI / 180);
  const lat2 = b.lat * (Math.PI / 180);
  const dLat = (b.lat - a.lat) * (Math.PI / 180);
  const dLon = (b.lon - a.lon) * (Math.PI / 180);
  const sinLat = Math.sin(dLat / 2);
  const sinLon = Math.sin(dLon / 2);
  const aCalc = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
  const c = 2 * Math.atan2(Math.sqrt(aCalc), Math.sqrt(1 - aCalc));
  return earthRadiusM * c;
}

function quantizeHeadingDeg(value) {
  const normalized = ((Number(value) % 360) + 360) % 360;
  return Math.round(normalized / 5) * 5;
}

function buildCoverageArrows(points = [], options = {}) {
  if (!Array.isArray(points) || points.length < 2) return [];

  const spacingM = Number.isFinite(Number(options.spacingM)) && Number(options.spacingM) > 0
    ? Number(options.spacingM)
    : 8;
  const initialOffsetM = Number.isFinite(Number(options.initialOffsetM)) && Number(options.initialOffsetM) >= 0
    ? Number(options.initialOffsetM)
    : Math.min(0.8, spacingM * 0.2);
  const minArrowSeparationM = Number.isFinite(Number(options.minArrowSeparationM)) && Number(options.minArrowSeparationM) > 0
    ? Number(options.minArrowSeparationM)
    : Math.max(3.2, spacingM * 0.55);

  const segments = [];
  for (let i = 0; i < points.length - 1; i++) {
    const from = points[i];
    const to = points[i + 1];
    if (!from || !to) continue;
    if (!Number.isFinite(from.lat) || !Number.isFinite(from.lon) || !Number.isFinite(to.lat) || !Number.isFinite(to.lon)) continue;
    const lenM = haversineDistanceMeters(from, to);
    if (!Number.isFinite(lenM) || lenM <= 0.01) continue;
    segments.push({ from, to, lenM });
  }

  if (!segments.length) return [];

  const totalLenM = segments.reduce((sum, segment) => sum + segment.lenM, 0);
  let segmentIndex = 0;
  let segmentStartDistM = 0;
  const arrows = [];

  for (let targetDistM = initialOffsetM; targetDistM < totalLenM; targetDistM += spacingM) {
    while (
      segmentIndex < segments.length - 1
      && (segmentStartDistM + segments[segmentIndex].lenM) < targetDistM
    ) {
      segmentStartDistM += segments[segmentIndex].lenM;
      segmentIndex += 1;
    }

    const segment = segments[segmentIndex];
    const distIntoSegmentM = Math.max(0, targetDistM - segmentStartDistM);
    const t = Math.max(0, Math.min(1, distIntoSegmentM / segment.lenM));
    const heading = Number.isFinite(segment.from.headingDeg)
      ? segment.from.headingDeg
      : headingDeg(segment.from, segment.to);

    const nextArrow = {
      lat: segment.from.lat + ((segment.to.lat - segment.from.lat) * t),
      lon: segment.from.lon + ((segment.to.lon - segment.from.lon) * t),
      headingDeg: quantizeHeadingDeg(heading),
    };

    const touchesExisting = arrows.some((existing) => haversineDistanceMeters(existing, nextArrow) < minArrowSeparationM);
    if (touchesExisting) continue;
    arrows.push(nextArrow);
  }

  if (!arrows.length) {
    const from = segments[0].from;
    const to = segments[0].to;
    const heading = Number.isFinite(from.headingDeg)
      ? from.headingDeg
      : headingDeg(from, to);
    arrows.push({
      lat: (from.lat + to.lat) * 0.5,
      lon: (from.lon + to.lon) * 0.5,
      headingDeg: quantizeHeadingDeg(heading),
    });
  }

  return arrows;
}

function withHeadings(points) {
  return points.map((point, index) => {
    const next = points[index + 1] ?? null;
    return {
      ...point,
      headingDeg: next ? Number(headingDeg(point, next).toFixed(1)) : null,
    };
  });
}

function findPath(map, startLatLon, goalLatLon) {
  let start = worldToGrid(map, startLatLon);
  let goal = worldToGrid(map, goalLatLon);

  if (!isWalkable(map, start.row, start.col)) {
    const snappedStart = nearestWalkable(map, start.row, start.col);
    if (!snappedStart) {
      return { ok: false, reason: 'Start is outside mapped boundary and no valid start cell found', points: [] };
    }
    start = snappedStart;
  }

  if (!isWalkable(map, goal.row, goal.col)) {
    const snappedGoal = nearestWalkable(map, goal.row, goal.col);
    if (!snappedGoal) {
      return { ok: false, reason: 'Goal is outside mapped boundary and no valid goal cell found', points: [] };
    }
    goal = snappedGoal;
  }

  const gridPath = findGridPath(map, start, goal);
  if (gridPath) {
    return {
      ok: true,
      reason: null,
      points: withHeadings(gridPath.map((point) => gridToWorld(map, point.row, point.col))),
    };
  }

  return { ok: false, reason: 'No path found', points: [] };
}

function findCoveragePath(map, options = {}) {
  const swathWidthM = Number.isFinite(Number(options.swathWidthM)) && Number(options.swathWidthM) > 0
    ? Number(options.swathWidthM)
    : map.cellSizeM;
  const startLatLon = options.startLatLon ?? null;
  const goalLatLon = options.goalLatLon ?? null;
  const requestedSweepDirection = typeof options.sweepDirection === 'string'
    ? options.sweepDirection.trim().toLowerCase()
    : 'auto';

  const rowStep = Math.max(1, Math.round(swathWidthM / map.cellSizeM));
  const laneStep = rowStep;
  const visitNodes = [];
  let prevNode = null;
  let forward = true;

  const axisUEastComponent = Math.abs(Number(map?.axisU?.x ?? 1));
  const axisVEastComponent = Math.abs(Number(map?.axisV?.x ?? 0));
  const sweepByRows = axisUEastComponent >= axisVEastComponent;

  let rowStart = 0;
  let rowLimit = map.height;
  let rowIncrement = laneStep;
  let colStart = 0;
  let colLimit = map.width;
  let colIncrement = laneStep;

  if (startLatLon && goalLatLon) {
    const startGrid = worldToGrid(map, startLatLon);
    const goalGrid = worldToGrid(map, goalLatLon);
    const snappedStart = nearestWalkable(map, startGrid.row, startGrid.col);
    const snappedGoal = nearestWalkable(map, goalGrid.row, goalGrid.col);

    if (snappedStart && snappedGoal) {
      if (sweepByRows) {
        const topAlignedRow = 0;
        const bottomAlignedRow = Math.max(0, map.height - 1 - ((map.height - 1) % laneStep));
        const rowDirection = snappedGoal.row >= snappedStart.row ? 1 : -1;

        rowStart = rowDirection > 0 ? topAlignedRow : bottomAlignedRow;
        rowLimit = rowDirection > 0 ? map.height : -1;
        rowIncrement = rowDirection > 0 ? laneStep : -laneStep;
        forward = snappedGoal.col >= snappedStart.col;
      } else {
        const leftAlignedCol = 0;
        const rightAlignedCol = Math.max(0, map.width - 1 - ((map.width - 1) % laneStep));
        const colDirection = snappedGoal.col >= snappedStart.col ? 1 : -1;

        colStart = colDirection > 0 ? leftAlignedCol : rightAlignedCol;
        colLimit = colDirection > 0 ? map.width : -1;
        colIncrement = colDirection > 0 ? laneStep : -laneStep;
        forward = snappedGoal.row >= snappedStart.row;
      }
    }
  }

  if (requestedSweepDirection === 'lefttoright' || requestedSweepDirection === 'righttoleft') {
    const eastAlongRunAxis = sweepByRows
      ? Number(map?.axisU?.x ?? 1)
      : Number(map?.axisV?.x ?? 0);
    const forwardForLeftToRight = eastAlongRunAxis >= 0;
    forward = requestedSweepDirection === 'lefttoright' ? forwardForLeftToRight : !forwardForLeftToRight;
  }

  if (sweepByRows) {
    for (let row = rowStart; row !== rowLimit; row += rowIncrement) {
      const insideCols = [];
      for (let col = 0; col < map.width; col++) {
        if (map.cells[row][col].inside) insideCols.push(col);
      }
      if (!insideCols.length) continue;

      const runs = splitContiguous(insideCols);
      const orderedRuns = forward ? runs : runs.slice().reverse();

      for (const run of orderedRuns) {
        const startCol = forward ? run.start : run.end;
        const endCol = forward ? run.end : run.start;
        const step = startCol <= endCol ? 1 : -1;
        const firstNode = { row, col: startCol };

        if (prevNode) {
          const connector = findGridPath(map, prevNode, firstNode);
          if (connector && connector.length > 1) {
            for (let i = 1; i < connector.length; i++) {
              visitNodes.push(connector[i]);
            }
          }
        } else {
          visitNodes.push(firstNode);
        }

        for (let col = startCol + step; ; col += step) {
          visitNodes.push({ row, col });
          if (col === endCol) break;
        }

        prevNode = { row, col: endCol };
        forward = !forward;
      }
    }
  } else {
    for (let col = colStart; col !== colLimit; col += colIncrement) {
      const insideRows = [];
      for (let row = 0; row < map.height; row++) {
        if (map.cells[row][col].inside) insideRows.push(row);
      }
      if (!insideRows.length) continue;

      const runs = splitContiguous(insideRows);
      const orderedRuns = forward ? runs : runs.slice().reverse();

      for (const run of orderedRuns) {
        const startRow = forward ? run.start : run.end;
        const endRow = forward ? run.end : run.start;
        const step = startRow <= endRow ? 1 : -1;
        const firstNode = { row: startRow, col };

        if (prevNode) {
          const connector = findGridPath(map, prevNode, firstNode);
          if (connector && connector.length > 1) {
            for (let i = 1; i < connector.length; i++) {
              visitNodes.push(connector[i]);
            }
          }
        } else {
          visitNodes.push(firstNode);
        }

        for (let row = startRow + step; ; row += step) {
          visitNodes.push({ row, col });
          if (row === endRow) break;
        }

        prevNode = { row: endRow, col };
        forward = !forward;
      }
    }
  }

  if (!visitNodes.length) {
    return { ok: false, reason: 'No interior cells available for coverage planning', points: [] };
  }

  return {
    ok: true,
    reason: null,
    points: withHeadings(visitNodes.map((node) => gridToWorld(map, node.row, node.col))),
    meta: {
      swathWidthM,
      rowStep,
      sweepByRows,
      sweepDirection: requestedSweepDirection,
      pointCount: visitNodes.length,
    },
  };
}

module.exports = {
  findPath,
  findCoveragePath,
  buildCoverageArrows,
};
