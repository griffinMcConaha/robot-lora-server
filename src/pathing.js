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

  const rowStep = Math.max(1, Math.round(swathWidthM / map.cellSizeM));
  const visitNodes = [];
  let prevNode = null;
  let forward = true;

  for (let row = 0; row < map.height; row += rowStep) {
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
      pointCount: visitNodes.length,
    },
  };
}

module.exports = {
  findPath,
  findCoveragePath,
};
