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

function reconstructPath(cameFrom, current) {
  const result = [current];
  let cursor = current;
  while (cameFrom.has(key(cursor.row, cursor.col))) {
    cursor = cameFrom.get(key(cursor.row, cursor.col));
    result.push(cursor);
  }
  return result.reverse();
}

function findPath(map, startLatLon, goalLatLon) {
  const start = worldToGrid(map, startLatLon);
  const goal = worldToGrid(map, goalLatLon);

  if (!isWalkable(map, start.row, start.col) || !isWalkable(map, goal.row, goal.col)) {
    return { ok: false, reason: 'Start or goal is outside mapped boundary', points: [] };
  }

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
      const gridPath = reconstructPath(cameFrom, current);
      return {
        ok: true,
        reason: null,
        points: gridPath.map((point) => gridToWorld(map, point.row, point.col)),
      };
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

  return { ok: false, reason: 'No path found', points: [] };
}

module.exports = {
  findPath,
};
