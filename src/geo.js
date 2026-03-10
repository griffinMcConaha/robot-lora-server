function degToRad(value) {
  return (value * Math.PI) / 180;
}

function metersPerDegLat() {
  return 111_320;
}

function metersPerDegLon(latDeg) {
  return 111_320 * Math.cos(degToRad(latDeg));
}

function latLonToLocal(point, origin) {
  const mx = (point.lon - origin.lon) * metersPerDegLon(origin.lat);
  const my = (point.lat - origin.lat) * metersPerDegLat();
  return { x: mx, y: my };
}

function localToLatLon(localPoint, origin) {
  return {
    lat: origin.lat + localPoint.y / metersPerDegLat(),
    lon: origin.lon + localPoint.x / metersPerDegLon(origin.lat),
  };
}

function pointInPolygon(point, polygon) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x;
    const yi = polygon[i].y;
    const xj = polygon[j].x;
    const yj = polygon[j].y;

    const intersects =
      yi > point.y !== yj > point.y &&
      point.x < ((xj - xi) * (point.y - yi)) / (yj - yi + Number.EPSILON) + xi;

    if (intersects) inside = !inside;
  }
  return inside;
}

module.exports = {
  latLonToLocal,
  localToLatLon,
  pointInPolygon,
};
