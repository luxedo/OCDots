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
import { simplify } from "./simplify.js";

// Default values
export const DEFAULTMASS = 1;
export const DEFAULTCHARGE = 1;
export const BASEFORCE = 4;
export const DRAG = 0.1;
export const VISCOSITY = 0.1;
export const MAXMOMENTUM = 5;
export const PARALLELFORCES = true;
export const WALLFORCES = 1;
export const ATTENUATION = 0.001;
export const SIMPLIFYPOLYGON = 0;
const GEO_MILIMITER_PRECISION = 100000000;
const polyCache = new WeakMap();
const sCache = new WeakMap();

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
 * polygon.
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
 * @param {Number[] | Number=} config.mass - Mass or masses of the points.
 * @param {Number[] | Number=} config.charge - Charge or charges of the points.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.simplifyPolygon - Simplify polygon tolerance (0 disabled)
 *
 * @return {Array[]} points, momentum - Updated points and momentum
 *      arrays
 */
export function movePoints({
  points,
  momentum,
  polygon,
  mass = DEFAULTMASS,
  charge = DEFAULTCHARGE,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  simplifyPolygon = SIMPLIFYPOLYGON,
}: {
  points: VecArray<vec>;
  momentum: VecArray<vec>;
  polygon: PolygonArray<vec>;
  mass?: number[] | number;
  charge?: number[] | number;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  simplifyPolygon?: number;
}): [VecArray<vec>, VecArray<vec>] {
  // Loads/sets cache
  let S: number, poly: PolygonArray<vec>;
  if (!polyCache.has(polygon)) {
    if (polygon.length < 3) {
      throw new RangeError("Polygon must have at least 3 vertices");
    }
    poly = <PolygonArray<vec>>[...polygon];
    if (simplifyPolygon != 0) poly = <PolygonArray<vec>>simplify(
        poly.map((pt) => {
          return { x: pt[0], y: pt[1] };
        }),
        simplifyPolygon,
        false
      ).map((pt) => {
        return <vec>[pt.x, pt.y];
      });
    if (
      poly[0][0] != poly[poly.length - 1][0] ||
      poly[0][1] != poly[poly.length - 1][1]
    ) {
      poly.push(poly[0]);
    }
    S = polygon.reduce((acc, cur) => {
      return acc + Math.sqrt(Math.pow(cur[0], 2) + Math.pow(cur[1], 2));
    }, 0);
    polyCache.set(polygon, poly);
    sCache.set(polygon, S);
  } else {
    poly = polyCache.get(polygon);
    S = sCache.get(polygon);
  }

  let p = <VecArray<vec>>[...points];
  let m = <VecArray<vec>>[...momentum];
  const _mass: number[] = Array.isArray(mass)
    ? mass
    : new Array(p.length).fill(mass);
  if (_mass.includes(0)) throw new RangeError("Mass cannot be zero.");
  const _charge: number[] = Array.isArray(charge)
    ? charge
    : new Array(p.length).fill(charge);
  const N = points.length;

  for (let i = 0; i < p.length; i++) {
    // Calculate forces
    const pt = points[i];
    const pf = pointForces(pt, _charge[i], points, _charge);
    const bf = polygonForces(pt, poly, parallelForces);
    const force: vec = [
      Math.pow(10, baseForce) *
        (pf[0] + (Math.abs(_charge[i]) * wallForces * N * bf[0]) / S),
      Math.pow(10, baseForce) *
        (pf[1] + (Math.abs(_charge[i]) * wallForces * N * bf[1]) / S),
      // Polygon forces are normalized to it's total length and
      // multiplied by wallForces
    ];

    // Update momentum
    const mx = force[0] / _mass[i] + m[i][0];
    const my = force[1] / _mass[i] + m[i][1];
    const norm = Math.sqrt(Math.pow(mx, 2) + Math.pow(my, 2));
    let mf = norm * (1 - drag);
    mf =
      mf < maxMomentum
        ? mf
        : maxMomentum *
          (Math.exp(-viscosity * (mf - maxMomentum)) * viscosity +
            (1 - viscosity));
    m[i] = [(mf * mx) / norm, (mf * my) / norm];

    // Update position
    for (let j = 0; j < 10; j++) {
      // We move points as close to the polygon as possible
      const px = pt[0] + m[i][0];
      const py = pt[1] + m[i][1];
      if (checkInbounds([px, py], poly)) {
        p[i] = [px, py];
        break;
      }
      // Lose momentum if colliding into the walls
      m[i][0] /= j + 2;
      m[i][1] /= j + 2;
      if (isNaN(m[i][0]) || isNaN(m[i][1])) {
        m[i][0] = 0;
        m[i][1] = 0;
      }
    }
  }

  return [<VecArray<vec>>p, <VecArray<vec>>m];
}

/**
 * Calculates forces on pt from points.
 * @private
 *
 * @param {Array} pt The point to measure forces
 * @param {number} c The point charge
 * @param {Array} points The points acting on pt
 * @param {Array} charge Array of other points charges
 *
 * @return {Array} force Sum of forces acting on pt
 */
export function pointForces(
  pt: vec,
  c: number,
  points: VecArray<vec>,
  charge: number[]
): vec {
  const f: vec = [0, 0];
  for (let i = 0; i < points.length; i++) {
    const vdx = pt[0] - points[i][0];
    const vdy = pt[1] - points[i][1];
    let norm2 = Math.pow(vdx, 2) + Math.pow(vdy, 2);
    norm2 = norm2 != 0 ? norm2 : Infinity;
    const norm = Math.sqrt(norm2);
    f[0] += (c * charge[i] * vdx) / norm / norm2;
    f[1] += (c * charge[i] * vdy) / norm / norm2;
  }
  return f;
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
 *
 * @return {Array} force Sum of forces acting on pt
 */
export function polygonForces(
  pt: vec,
  polygon: PolygonArray<vec>,
  parallelForces: boolean = PARALLELFORCES
): vec {
  const f: vec = [0, 0];
  for (let i = 1; i < polygon.length; i++) {
    // Useful vectors
    const a0 = polygon[i - 1]; // a from origin
    const b0 = polygon[i]; // b from origin
    const t = perpendicularToLine(pt, a0, b0);
    const y2 = Math.pow(t[0], 2) + Math.pow(t[1], 2);
    const y = Math.sqrt(y2);
    const normal = [b0[0] - a0[0], b0[1] - a0[1]];
    const n2 = Math.sqrt(Math.pow(normal[0], 2) + Math.pow(normal[1], 2));
    const xn = [normal[0] / n2, normal[1] / n2];
    const yn = [xn[1], -xn[0]];
    const apt = [a0[0] - pt[0], a0[1] - pt[1]];
    const bpt = [b0[0] - pt[0], b0[1] - pt[1]];

    // The Magic
    const thetaT = Math.atan2(t[1], t[0]);
    const thetaA = thetaT - Math.atan2(apt[1], apt[0]);
    const thetaB = thetaT - Math.atan2(bpt[1], bpt[0]);
    let thetaDiff = thetaB - thetaA; // Align thetaDiff between 0 and +PI
    thetaDiff = thetaDiff <= 0 ? thetaDiff + 2 * Math.PI : thetaDiff;
    thetaDiff = thetaDiff > Math.PI ? thetaDiff - 2 * Math.PI : thetaDiff;

    let Ex = 0; // Parallel fraction
    let Ey = 0; // Perpendicular fraction
    if (y != 0) {
      // Just in case. We pretend nothing happens if segment is colinear with point.
      Ex = parallelForces ? (Math.cos(thetaB) - Math.cos(thetaA)) / y : 0;
      Ey = (Math.sin(thetaB) - Math.sin(thetaA)) / y;
    }

    f[0] += Ey * yn[0] + Ex * xn[0];
    f[1] += Ey * yn[1] + Ex * xn[1];
  }
  return f;
}

/**
 * Returns the perpendicular line to segment v1 -> v2.
 * @private
 *
 * @param {Array} pt Point to calculate perpendicular to
 * @param {Array} v1 Line segment vertetx 1
 * @param {Array} v2 Line segment vertetx 2
 *
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
 * Tests if a point is Left|On|Right of an infinite line.
 * From: https://github.com/iominh/point-in-polygon-extended/blob/master/src/index.js
 *
 * See http://geomalgorithms.com/a01-_area.html
 *
 * @param {object} p0 x,y point
 * @param {object} p1 x,y point
 * @param {object} p2 x,y point
 *
 * @returns {number}
 *  >0 for P2 left of the line through P0 and P1,
 *  =0 for P2  on the line,
 *  <0 for P2  right of the line
 */
function isLeft(p0, p1, p2) {
  return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
}

/**
 * Checks if the point pt is inside polygon.
 * From: https://github.com/iominh/point-in-polygon-extended/blob/master/src/index.js
 *
 * https://en.wikipedia.org/wiki/Even%E2%80%93odd_rule
 * https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
 *
 * @private
 *
 * @param {Array} pt The point to measure forces
 * @param {Array} polygon Set of points that describes the polygon.
 *      This polygon should be ordered (clockwise or anticlockwise) and
 *      closed i.e. first points equals the last point.
 *
 * @return {boolean} inbound True if pt is inside the polygon
 */
export function checkInbounds(pt: vec, polygon: PolygonArray<vec>): boolean {
  if (polygon.length === 0) {
    return false;
  }

  const n = polygon.length;
  const newPoints = polygon.slice(0);
  newPoints.push(polygon[0]);
  let wn = 0; // wn counter

  // loop through all edges of the polygon
  for (let i = 0; i < n; i++) {
    if (newPoints[i][1] <= pt[1]) {
      if (newPoints[i + 1][1] > pt[1]) {
        if (isLeft(newPoints[i], newPoints[i + 1], pt) > 0) {
          wn++;
        }
      }
    } else {
      if (newPoints[i + 1][1] <= pt[1]) {
        if (isLeft(newPoints[i], newPoints[i + 1], pt) < 0) {
          wn--;
        }
      }
    }
  }
  // the pt is outside only when this winding number wn===0, otherwise it's inside
  return wn !== 0;
}

/**
 * Creates N points inside the polygon
 *
 * @param {Number} N Number of points
 * @param {Array} polygon Set of points that describes the polygon.
 *      This polygon should be ordered (clockwise or anticlockwise) and
 *      closed i.e. first points equals the last point.
 *
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
 *
 * @return {Array} points N points inside the geo polygon
 */
export function randomInGeoPolygon(
  N: number,
  geoPolygon: VecArray<geoVec>
): VecArray<geoVec> {
  return <VecArray<geoVec>>randomInPolygon(
    N,
    <PolygonArray<vec>>(
      geoPolygon.map(
        (p) =>
          <vec>[
            GEO_MILIMITER_PRECISION * p.lat,
            GEO_MILIMITER_PRECISION * p.lng,
          ]
      )
    )
  ).map(
    (p) =>
      <geoVec>{
        lat: p[0] / GEO_MILIMITER_PRECISION,
        lng: p[1] / GEO_MILIMITER_PRECISION,
      }
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
 * @param {Number[] | Number=} config.mass - Mass or masses of the points.
 * @param {Number[] | Number=} config.charge - Charge or charges of the points.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.simplifyPolygon - Simplify polygon tolerance (0 disabled)
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
  mass = DEFAULTMASS,
  charge = DEFAULTCHARGE,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  simplifyPolygon = SIMPLIFYPOLYGON,
  attenuation = ATTENUATION,
}: {
  points: VecArray<vec>;
  momentum: VecArray<vec>;
  polygon: PolygonArray<vec>;
  iterations: number;
  callback?: Function;
  mass?: number[] | number;
  charge?: number[] | number;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  simplifyPolygon?: number;
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
      mass,
      baseForce,
      drag: drag * attIter,
      viscosity,
      maxMomentum,
      parallelForces,
      wallForces,
      simplifyPolygon,
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
 * @param {Number[] | Number=} config.mass - Mass or masses of the points.
 * @param {Number[] | Number=} config.charge - Charge or charges of the points.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.simplifyPolygon - Simplify polygon tolerance (0 disabled)
 * @param {Number=} config.attenuation - Rate of attenuation
 *
 * @return {Array} points Last iteration points positions
 */

export function relaxNPoints({
  N,
  polygon,
  iterations,
  callback,
  mass = DEFAULTMASS,
  charge = DEFAULTCHARGE,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  simplifyPolygon = SIMPLIFYPOLYGON,
  attenuation = ATTENUATION,
}: {
  N: number;
  polygon: PolygonArray<vec>;
  iterations: number;
  callback?: Function;
  mass?: number[] | number;
  charge?: number[] | number;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  simplifyPolygon?: number;
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
    mass,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    simplifyPolygon,
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
 * @param {Number[] | Number=} config.mass - Mass or masses of the points.
 * @param {Number[] | Number=} config.charge - Charge or charges of the points.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.simplifyPolygon - Simplify polygon tolerance (0 disabled)
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
  mass = DEFAULTMASS,
  charge = DEFAULTCHARGE,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  simplifyPolygon = SIMPLIFYPOLYGON,
  attenuation = ATTENUATION,
}: {
  geoPoints: VecArray<geoVec>;
  geoPolygon: PolygonArray<geoVec>;
  width: number;
  iterations: number;
  callback?: Function;
  mass?: number[] | number;
  charge?: number[] | number;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  simplifyPolygon?: number;
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
    mass,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    simplifyPolygon,
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
 * @param {Number[] | Number=} config.mass - Mass or masses of the points.
 * @param {Number[] | Number=} config.charge - Charge or charges of the points.
 * @param {Number=} config.baseForce - The force constant
 * @param {Number=} config.drag - The drag coeficient
 * @param {Number=} config.viscosity - The viscosity coeficient
 * @param {Number=} config.maxMomentum - Maximum momentum for each point
 * @param {Boolean=} config.parallelForces - Sum line segmen parallel forces
 *    as well.
 * @param {Number=} config.wallForces - Walls forces constant
 * @param {Number=} config.simplifyPolygon - Simplify polygon tolerance (0 disabled)
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
  mass = DEFAULTMASS,
  charge = DEFAULTCHARGE,
  baseForce = BASEFORCE,
  drag = DRAG,
  viscosity = VISCOSITY,
  maxMomentum = MAXMOMENTUM,
  parallelForces = PARALLELFORCES,
  wallForces = WALLFORCES,
  simplifyPolygon = SIMPLIFYPOLYGON,
  attenuation = ATTENUATION,
}: {
  N: number;
  geoPolygon: PolygonArray<geoVec>;
  width: number;
  iterations: number;
  callback?: Function;
  mass?: number[] | number;
  charge?: number[] | number;
  baseForce?: number;
  drag?: number;
  viscosity?: number;
  maxMomentum?: number;
  parallelForces?: boolean;
  wallForces?: number;
  simplifyPolygon?: number;
  attenuation?: number;
}) {
  const geoPoints = randomInGeoPolygon(N, geoPolygon);
  return relaxGeoPoints({
    geoPoints,
    geoPolygon,
    width,
    iterations,
    mass,
    callback,
    baseForce,
    drag,
    viscosity,
    maxMomentum,
    parallelForces,
    wallForces,
    simplifyPolygon,
    attenuation,
  });
}

/**
 * Transforms a set of coordinates into a polygon with a known width
 *
 * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
 * @param {Number} width Width of the polygon
 *
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
  if (
    polygon[0][0] != polygon[polygon.length - 1][0] ||
    polygon[0][1] != polygon[polygon.length - 1][1]
  ) {
    polygon.push(polygon[0]);
  }
  return { polygon, minLat, minLng, delta };
}
