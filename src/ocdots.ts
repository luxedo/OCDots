/**
 * OCDots
 *
 * OCDots is a javascript library for creating evenly distributed
 * points inside a polygon
 *
 * Copyright (C) 2021 Luiz Eduardo Amaral <luizamaral306@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Default values
export const BASEFORCE = 5;
export const DRAG = 0.1;
export const VISCOSITY = 0.1;
export const MAXMOMENTUM = 5;
export const PARALLELFORCES = true;
export const WALLFORCES = 2;
export const ATTENUATION = 0.001;

// Types
type vec = [number, number];
type geoVec = { lat: number; lng: number };
type VecArray<T> = {
  0: T;
} & Array<T>; // Vector arrays must have at least one vector
type PolygonArray<T> = {
  0: T;
  1: T;
  2: T;
} & Array<T>; // Polygons must have at least 3 vectors

/**
 * Moves points according to the applied forces into it. The forces
 * are: 1) between points, 2) between the point and walls of the
 * polygon, 3) between points and vertexes of the polygon.
 *
 * The points moves according to it's momentum up to maxMomentum.
 * Drag reduces the momentum with the square of the momentum.
 * Viscosity lowers the momentum of points with high forces.
 *
 * Runs one iteration
 *
 * @param {Object} config - configuration object
 * @param {Array} congig.points - The points to move
 * @param {Array} config.momentum - Accumulated momentum for each point. [0, 0]
 *      when the points are stopped.
 * @param {Array} config.polygon - Set of points that describes the polygon that
 *      contains the points. This polygon should be ordered (clockwise
 *      or anticlockwise) and closed i.e. first points equals the last
 *      point.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @return {Array[]} points, momentum - Updated points and momentum
 *      arrays
 */
export function movePoints({
  points,
  momentum,
  polygon,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
}: {
  points: VecArray<vec>;
  momentum: VecArray<vec>;
  polygon: PolygonArray<vec>;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
}): [VecArray<vec>, VecArray<vec>] {
  if (polygon.length < 3) {
    throw new RangeError("Polygon must have at least 3 vertices");
  }
  let p = <VecArray<vec>>[...points];
  let m = <VecArray<vec>>[...momentum];
  const N = points.length;

  if (
    polygon[0][0] != polygon[polygon.length - 1][0] ||
    polygon[0][1] != polygon[polygon.length - 1][1]
  ) {
    polygon.push(polygon[0]);
  }

  const S = polygon.reduce((acc, cur) => {
    return acc + Math.sqrt(Math.pow(cur[0], 2) + Math.pow(cur[1], 2));
  }, 0);

  // Calculate forces
  const pf = p.map((pt) => pointForces(pt, p));
  const bf = p.map((pt) => polygonForces(pt, polygon, parallelForces));

  // Update momentum
  m = <VecArray<vec>>m.map((mt, i) => {
    const force: vec = [
      Math.pow(10, baseForce) * (pf[i][0] + (wallForces * N * bf[i][0]) / S),
      Math.pow(10, baseForce) * (pf[i][1] + (wallForces * N * bf[i][1]) / S),
      // Polygon forces are normalized to it's total length and
      // multiplied by wallForces
    ];
    return updateMomentum(mt, force, drag, viscosity, maxMomentum);
  });

  // Update position
  p = <VecArray<vec>>p.map((pt, i) => {
    for (let j = 0; j < 10; j++) {
      // We move points as close to the polygon as possible
      const px = pt[0] + m[i][0];
      const py = pt[1] + m[i][1];
      if (checkInbounds([px, py], polygon)) {
        return [px, py];
      }
      // Lose momentum if colliding into the walls
      m[i][0] /= i + 2;
      m[i][1] /= i + 2;
    }
    return pt;
  });
  return [<VecArray<vec>>p, <VecArray<vec>>m];
}

/**
 * Calculates forces on pt from points.
 * @private
 *
 * @param {Array} pt The point to measure forces
 * @param {Array} points The points acting on pt
 * @return {Array} force Sum of forces acting on pt
 */
export function pointForces(pt: vec, points: VecArray<vec>): vec {
  return points.reduce(
    (acc, pt0) => {
      const vdx = pt[0] - pt0[0];
      const vdy = pt[1] - pt0[1];
      let norm2 = Math.pow(vdx, 2) + Math.pow(vdy, 2);
      norm2 = norm2 != 0 ? norm2 : Infinity;
      const norm = Math.sqrt(norm2);
      const fx = vdx / norm / norm2;
      const fy = vdy / norm / norm2;
      return [acc[0] + fx, acc[1] + fy];
    },
    [0, 0]
  );
}

/**
 * Calculates forces on pt from polygon. The forces are the integral
 * of the forces for each line segment for a charged wire as in:
 * https://aapt.scitation.org/doi/full/10.1119/1.4906421
 *
 * The default behavior is to sum perpendicular and parallel forces
 * for each line segment, setting `parallelForces` to `false` sums
 * only perpendicular forces, but this may lead to instability
 * because of nonlinearity in the forces for concave polygons.
 * @private
 *
 * @param {Array} pt The point to measure forces
 * @param {Array} polygon The polygons vertexes
 * @param {Boolean} parallelForces Sum line segmen parallel forces
 *    as well.
 * @return {Array} force Sum of forces acting on pt
 */
export function polygonForces(
  pt: vec,
  polygon: PolygonArray<vec>,
  parallelForces: boolean = PARALLELFORCES
): vec {
  return polygon.slice(1).reduce(
    (acc, v2, i1) => {
      const v1 = polygon[i1];

      const t = perpendicularToLine(pt, v1, v2);
      const p = [-t[1], t[0]];
      const nt = Math.sqrt(Math.pow(t[0], 2) + Math.pow(t[1], 2));

      const v1pt = [pt[0] - v1[0], pt[1] - v1[1]];
      const v2pt = [pt[0] - v2[0], pt[1] - v2[1]];
      const thetaA = Math.atan2(v1pt[1], v1pt[0]) - Math.atan2(t[1], t[0]);
      const thetaB = Math.atan2(v2pt[1], v2pt[0]) - Math.atan2(t[1], t[0]);
      // Align thetaDiff between -PI and +PI
      let thetaDiff = thetaB - thetaA;
      thetaDiff = thetaDiff <= Math.PI ? thetaDiff + 2 * Math.PI : thetaDiff;
      thetaDiff = thetaDiff > Math.PI ? thetaDiff - 2 * Math.PI : thetaDiff;

      const modulus = (1 / nt) * Math.sin((1 / 2) * thetaDiff);

      const tv = Math.sin(thetaB) - Math.sin(thetaA); // Perpendicular fraction
      const pv = -(Math.cos(thetaB) - Math.cos(thetaA)); // Parallel fraction
      const dn = Math.sqrt(Math.pow(pv, 2) + Math.pow(tv, 2));

      const direction = parallelForces
        ? [
            (tv * t[0]) / nt / dn + (pv * p[0]) / nt / dn,
            (tv * t[1]) / nt / dn + (pv * p[1]) / nt / dn,
          ]
        : [(tv * t[0]) / nt / dn, (tv * t[1]) / nt / dn];
      const f = [modulus * direction[0], modulus * direction[1]];
      return [acc[0] + f[0], acc[1] + f[1]];
    },
    [0, 0]
  );
}

/**
 * Returns the perpendicular line to segment v1 -> v2.
 * @private
 *
 * @param {Array} pt Point to calculate perpendicular to
 * @param {Array} v1 Line segment vertetx 1
 * @param {Array} v2 Line segment vertetx 2
 * @return {Array} p Perpendicular vector
 */
export function perpendicularToLine(pt: vec, v1: vec, v2: vec): vec {
  const v1v2 = [v1[0] - v2[0], v1[1] - v2[1]];
  const nv1v2 = Math.sqrt(Math.pow(v1v2[0], 2) + Math.pow(v1v2[1], 2));
  const v1p = [v1[0] - pt[0], v1[1] - pt[1]];
  const n = [v1v2[0] / nv1v2, v1v2[1] / nv1v2];
  const d = v1p[0] * n[0] + v1p[1] * n[1];
  return [v1p[0] - d * n[0], v1p[1] - d * n[1]];
}

/**
 * Updates the momentum.
 * @private
 *
 * @param {Array} mt Initial momentum
 * @param {Array} force Forcing acting at the point
 * @param {Number} drag The drag coeficient
 * @param {Number} viscosity The viscosity coeficient
 * @param {Number} maxMomentum Maximum momentum for each point
 * @return {Array} mu Updated momentum
 */
export function updateMomentum(
  mt: vec,
  force: vec,
  drag: number,
  viscosity: number,
  maxMomentum: number
): vec {
  const mx = force[0] + mt[0];
  const my = force[1] + mt[1];
  const norm = Math.sqrt(Math.pow(mx, 2) + Math.pow(my, 2));
  let m = norm * (1 - drag);
  m =
    m < maxMomentum
      ? m
      : maxMomentum *
        (Math.exp(-viscosity * (m - maxMomentum)) * viscosity +
          (1 - viscosity));
  return [(m * mx) / norm, (m * my) / norm];
}

/**
 * Checks if the point pt is inside polygon.
 * @private
 *
 * @param {Array} pt The point to measure forces
 * @param {Array} polygon Set of points that describes the polygon.
 *      This polygon should be ordered (clockwise or anticlockwise) and
 *      closed i.e. first points equals the last point.
 * @return {boolean} inbound True if pt is inside the polygon
 */
export function checkInbounds(pt: vec, polygon: PolygonArray<vec>): boolean {
  let inbound = false;
  const [x, y] = pt;
  for (let i = 1; i < polygon.length; i++) {
    const [p1x, p1y] = polygon[i - 1];
    const [p2x, p2y] = polygon[i];
    if (p2y > y != p1y > y && x < p2x + ((p1x - p2x) * (y - p2y)) / (p1y - p2y))
      inbound = !inbound;
  }
  return inbound;
}

/**
 * Creates N points inside the polygon
 *
 * @param {Number} N Number of points
 * @param {Array} polygon Set of points that describes the polygon.
 *      This polygon should be ordered (clockwise or anticlockwise) and
 *      closed i.e. first points equals the last point.
 * @return {Array} points N points inside the polygon
 */
export function randomInPolygon(
  N: number,
  polygon: PolygonArray<vec>
): VecArray<vec> {
  let points = [];
  const { xMin, xMax, yMin, yMax } = polygon.reduce(
    (acc, v) => {
      acc.xMin = v[0] < acc.xMin ? v[0] : acc.xMin;
      acc.xMax = v[0] > acc.xMax ? v[0] : acc.xMax;
      acc.yMin = v[1] < acc.yMin ? v[1] : acc.yMin;
      acc.yMax = v[1] > acc.yMax ? v[1] : acc.yMax;
      return acc;
    },
    {
      xMin: Infinity,
      xMax: -Infinity,
      yMin: Infinity,
      yMax: -Infinity,
    }
  );
  const deltaX = xMax - xMin;
  const deltaY = yMax - yMin;
  while (points.length < N) {
    const pt: vec = [
      xMin + Math.random() * deltaX,
      yMin + Math.random() * deltaY,
    ];
    if (checkInbounds(pt, polygon)) points.push(pt);
  }
  return <VecArray<vec>>points;
}

/**
 * Creates N points inside a geo polygon,
 *
 * @param {Number} N Number of points
 * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
 * @return {Array} points N points inside the geo polygon
 */
export function randomInGeoPolygon(
  N: number,
  geoPolygon: VecArray<geoVec>
): VecArray<geoVec> {
  return <VecArray<geoVec>>(
    randomInPolygon(
      N,
      <PolygonArray<vec>>geoPolygon.map((p) => <vec>[p.lat, p.lng])
    ).map((p) => <geoVec>{ lat: p[0], lng: p[1] })
  );
}

/**
 * Runs several iterations of movePoints(). The drag increases in every
 * iteration attenuating the movement.
 *
 * @param {Object} config - configuration object
 * @param {Array} config.points - The points to move
 * @param {Array} config.momentum - Initial momentum. [0, 0] for all points if
 *      ommited
 * @param {Array} config.polygon - Set of points that describes the polygon that
 *      contains the points. This polygon should be ordered (clockwise
 *      or anticlockwise) and closed i.e. first points equals the last
 *      point.
 * @param {Number} config.iterations - Number of iterations to run
 * @param {Function=} config.callback - Callback function to run at every
 *      iteration (optional). Callback args: points, momentum, polygon,
 *      baseForce, currentDrag, viscosity, maxMomentum
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.attenuation - Rate of attenuation
 *
 * @return {Array} points Last iteration points positions
 */
export function relaxPoints({
  points,
  momentum,
  polygon,
  iterations,
  callback,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  attenuation = ATTENUATION,
}: {
  points: VecArray<vec>;
  momentum: VecArray<vec>;
  polygon: PolygonArray<vec>;
  iterations: number;
  callback?: Function;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  attenuation?: number;
}) {
  let p: VecArray<vec> = <VecArray<vec>>[...points];
  let m: VecArray<vec> = <VecArray<vec>>(
    (momentum == undefined ? p.map(() => [0, 0]) : [...momentum])
  );

  let att = 1 + attenuation;
  for (let i = 0, attIter = att; i < iterations; i++, attIter *= att) {
    [p, m] = movePoints({
      points: p,
      momentum: m,
      polygon,
      baseForce,
      drag: drag * attIter,
      viscosity,
      maxMomentum,
      parallelForces,
      wallForces,
    });
    if (callback != undefined) {
      callback(
        p,
        m,
        polygon,
        baseForce,
        drag * attIter,
        viscosity,
        maxMomentum
      );
    }
  }
  return p;
}

/**
 * Calls relaxPoints for N random points placed inside the polygon.
 *
 * @param {Object} config - configuration object
 * @param {Number} config.N - Number of points
 * @param {Array} config.polygon - Set of points that describes the polygon that
 *      contains the points. This polygon should be ordered (clockwise
 *      or anticlockwise) and closed i.e. first points equals the last
 *      point.
 * @param {Number} config.iterations - Number of iterations to run
 * @param {Function=} config.callback - Callback function to run at every
 *      iteration (optional). Callback args: points, momentum, polygon,
 *      baseForce, currentDrag, viscosity, maxMomentum
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.attenuation - Rate of attenuation
 *
 * @return {Array} points Last iteration points positions
 */

export function relaxNPoints({
  N,
  polygon,
  iterations,
  callback,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  attenuation = ATTENUATION,
}: {
  N: number;
  polygon: PolygonArray<vec>;
  iterations: number;
  callback?: Function;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  attenuation?: number;
}) {
  const points = randomInPolygon(N, polygon);
  const momentum: VecArray<vec> = <VecArray<vec>>points.map(() => [0, 0]);
  return relaxPoints({
    points,
    momentum,
    polygon,
    iterations,
    callback,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    attenuation,
  });
}

/**
 * Converts geoPoints and geoPolygon to a points and polygon, then
 * calls relaxPoints, returning the last position of the points.
 *
 * @param {Object} config - configuration object
 * @param {Array} config.geoPoints - Points in geo coordinates {lat, lng}
 * @param {Array} config.geoPolygon - Polygon of geo coordinates {lat, lng}
 * @param {Number} config.width - Width of the polygon
 * @param {Number} config.iterations - Number of iterations to run
 * @param {Function=} confi.callback - Callback function to run at every
 *      iteration. Callback args: points, momentum, polygon, baseForce,
 *      currentDrag, viscosity, maxMomentum
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.attenuation - Rate of attenuation
 *
 * @return {Object} { polygon, points, geoPoints } Last iteration geo
 *      points positions
 */
export function relaxGeoPoints({
  geoPoints,
  geoPolygon,
  width,
  iterations,
  callback,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  attenuation = ATTENUATION,
}: {
  geoPoints: VecArray<geoVec>;
  geoPolygon: PolygonArray<geoVec>;
  width: number;
  iterations: number;
  callback?: Function;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  attenuation?: number;
}) {
  const { polygon, minLat, minLng, delta } = buildPolygon(geoPolygon, width);

  const momentum: VecArray<vec> = <VecArray<vec>>geoPoints.map(() => [0, 0]);
  const points = <VecArray<vec>>relaxPoints({
    points: <VecArray<vec>>(
      geoPoints.map(
        (p) => <vec>[(p.lat - minLat) * delta, (p.lng - minLng) * delta]
      )
    ),
    momentum,
    polygon,
    iterations,
    callback,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    attenuation,
  });
  return {
    polygon,
    points,
    geoPoints: points.map((pt) => ({
      lat: pt[0] / delta + minLat,
      lng: pt[1] / delta + minLng,
    })),
  };
}

/**
 * Calls relaxGeoPoints for N random points placed inside the polygon.
 *
 * @param {Object} config - configuration object
 * @param {Number} config.N - Number of points
 * @param {Array} config.geoPolygon - Polygon of geo coordinates {lat, lng}
 * @param {Number} config.width - Width of the polygon
 * @param {Number} config.iterations - jNumber of iterations to run
 * @param {Function=} config.callback - Callback function to run at every
 *      iteration. Callback args: points, momentum, polygon, baseForce,
 *      currentDrag, viscosity, maxMomentum
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.attenuation - Rate of attenuation
 *
 * @return {Object} { polygon, points, geoPoints } Last iteration geo
 *      points positions
 */
export function relaxNGeoPoints({
  N,
  geoPolygon,
  width,
  iterations,
  callback,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  attenuation = ATTENUATION,
}: {
  N: number;
  geoPolygon: PolygonArray<geoVec>;
  width: number;
  iterations: number;
  callback?: Function;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  attenuation?: number;
}) {
  const geoPoints = randomInGeoPolygon(N, geoPolygon);
  return relaxGeoPoints({
    geoPoints,
    geoPolygon,
    width,
    iterations,
    callback,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    attenuation,
  });
}

/**
 * Transforms a set of coordinates into a polygon with a known width
 *
 * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
 * @param {Number} width Width of the polygon
 * @return {Object} { polygon, minLat, minLng, delta }
 */
export function buildPolygon(
  geoPolygon: VecArray<geoVec>,
  width: number
): {
  polygon: PolygonArray<vec>;
  minLat: number;
  minLng: number;
  delta: number;
} {
  const { minLat, maxLat, minLng, maxLng } = geoPolygon.reduce(
    (acc, v) => {
      acc.minLat = v.lat < acc.minLat ? v.lat : acc.minLat;
      acc.maxLat = v.lat > acc.maxLat ? v.lat : acc.maxLat;
      acc.minLng = v.lng < acc.minLng ? v.lng : acc.minLng;
      acc.maxLng = v.lng > acc.maxLng ? v.lng : acc.maxLng;
      return acc;
    },
    {
      minLat: Infinity,
      maxLat: -Infinity,
      minLng: Infinity,
      maxLng: -Infinity,
    }
  );
  const delta = width / (maxLng - minLng);
  let polygon: PolygonArray<vec> = <PolygonArray<vec>>(
    geoPolygon.map(
      (v) => <vec>[delta * (v.lat - minLat), delta * (v.lng - minLng)]
    )
  );
  // polygon = sortPolygon(polygon);
  if (
    polygon[0][0] != polygon[polygon.length - 1][0] ||
    polygon[0][1] != polygon[polygon.length - 1][1]
  ) {
    polygon.push(polygon[0]);
  }
  return { polygon, minLat, minLng, delta };
}

/**
 * Sorts the polygon vertexes.
 * @private
 *
 * @param {Array} polygon Set of points that describes the polygon
 * @return {Array} polygon Sorted polygon
 */
export function sortPolygon(polygon) {
  const centroid = polygon.reduce(
    (acc, v, i, arr) => [
      acc[0] + v[0] / arr.length,
      acc[1] + v[1] / arr.length,
    ],
    [0, 0]
  );
  polygon = polygon.sort(
    (a, b) =>
      Math.atan2(b[1] - centroid[1], b[0] - centroid[0]) -
      Math.atan2(a[1] - centroid[1], a[0] - centroid[0])
  );
  polygon.push(polygon[0]);
  return polygon;
}

/**
 * Plots a polygon in the canvas
 *
 * @param {Object} ctx Canvas context
 * @param {Array} polygon Set of points that describes the polygon
 * @param {String} color Points color
 */
export function drawPolygon(ctx, polygon, color = "black") {
  ctx.strokestyle = color;
  const pt0 = polygon[0];
  ctx.beginPath();
  ctx.moveTo(pt0[0], pt0[1]);
  polygon.slice(1).forEach((pt) => {
    ctx.lineTo(pt[0], pt[1]);
  });
  ctx.closePath();
  ctx.stroke();
}

/**
 * Plots points in the canvas
 *
 * @param {Object} ctx Canvas context
 * @param {Array} polygon Set of points that describes the polygon
 * @param {Number} radius points radius
 * @param {String} color Points color
 */
export function drawPoints(ctx, points, radius = 6, color = "black") {
  ctx.fillStyle = color;
  points.forEach((pt) => {
    ctx.beginPath();
    ctx.arc(pt[0], pt[1], radius, 0, 2 * Math.PI, 0);
    ctx.fill();
  });
}

/**
 * Clears the canvas and sets a background color
 *
 * @param {Object} canvas Canvas object
 * @param {String} backgroundColor Points color
 */
export function resetCanvas(canvas, backgroundColor = "white") {
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = backgroundColor;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
}

/**
 * Plots the points and polygon into a blank canvas
 *
 * @param {Object} canvas Canvas object
 * @param {Array} polygon Set of points that describes the polygon
 * @param {Number} radius points radius
 * @param {String} color Points color
 * @param {String} backgroundColor Points color
 */
export function drawPolygonAndPoints(
  canvas,
  points,
  polygon,
  radius = 6,
  color = "black",
  backgroundColor = "white"
) {
  const ctx = canvas.getContext("2d");
  const { width, height } = polygon.reduce(
    (acc, v) => {
      acc.width = acc.width < v[0] ? v[0] : acc.width;
      acc.height = acc.height < v[1] ? v[1] : acc.height;
      return acc;
    },
    { width: 0, height: 0 }
  );
  resetCanvas(canvas, backgroundColor);
  drawPoints(ctx, points, radius, color);
  drawPolygon(ctx, polygon, color);
}
