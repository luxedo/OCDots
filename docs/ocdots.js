var __spreadArray =
  (this && this.__spreadArray) ||
  function (to, from) {
    for (var i = 0, il = from.length, j = to.length; i < il; i++, j++) to[j] = from[i];
    return to;
  };
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
import { simplify } from './simplify.js';
// Default values
export var DEFAULTMASS = 1;
export var DEFAULTCHARGE = 1;
export var BASEFORCE = 4;
export var DRAG = 0.1;
export var VISCOSITY = 0.1;
export var MAXMOMENTUM = 5;
export var PARALLELFORCES = true;
export var WALLFORCES = 1;
export var ATTENUATION = 0.001;
export var SIMPLIFYPOLYGON = 0;
var GEO_MILIMITER_PRECISION = 100000000;
var polyCache = new WeakMap();
var sCache = new WeakMap();
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
export function movePoints(_a) {
  var points = _a.points,
    momentum = _a.momentum,
    polygon = _a.polygon,
    _b = _a.mass,
    mass = _b === void 0 ? DEFAULTMASS : _b,
    _c = _a.charge,
    charge = _c === void 0 ? DEFAULTCHARGE : _c,
    _d = _a.baseForce,
    baseForce = _d === void 0 ? BASEFORCE : _d,
    _e = _a.drag,
    drag = _e === void 0 ? DRAG : _e,
    _f = _a.viscosity,
    viscosity = _f === void 0 ? VISCOSITY : _f,
    _g = _a.maxMomentum,
    maxMomentum = _g === void 0 ? MAXMOMENTUM : _g,
    _h = _a.parallelForces,
    parallelForces = _h === void 0 ? PARALLELFORCES : _h,
    _j = _a.wallForces,
    wallForces = _j === void 0 ? WALLFORCES : _j,
    _k = _a.simplifyPolygon,
    simplifyPolygon = _k === void 0 ? SIMPLIFYPOLYGON : _k;
  // Loads/sets cache
  var S;
  var poly;
  if (!polyCache.has(polygon)) {
    if (polygon.length < 3) {
      throw new RangeError('Polygon must have at least 3 vertices');
    }
    poly = __spreadArray([], polygon);
    if (simplifyPolygon != 0) {
      poly = simplify(
        poly.map(function (pt) {
          return { x: pt[0], y: pt[1] };
        }),
        simplifyPolygon,
        false
      ).map(function (pt) {
        return [pt.x, pt.y];
      });
    }
    if (poly[0][0] != poly[poly.length - 1][0] || poly[0][1] != poly[poly.length - 1][1]) {
      poly.push(poly[0]);
    }
    S = polygon.reduce(function (acc, cur) {
      return acc + Math.sqrt(Math.pow(cur[0], 2) + Math.pow(cur[1], 2));
    }, 0);
    polyCache.set(polygon, poly);
    sCache.set(polygon, S);
  } else {
    poly = polyCache.get(polygon);
    S = sCache.get(polygon);
  }
  var p = __spreadArray([], points);
  var m = __spreadArray([], momentum);
  var _mass = Array.isArray(mass) ? mass : new Array(p.length).fill(mass);
  if (_mass.includes(0)) {
    throw new RangeError('Mass cannot be zero.');
  }
  var _charge = Array.isArray(charge) ? charge : new Array(p.length).fill(charge);
  var N = points.length;
  for (var i = 0; i < p.length; i++) {
    // Calculate forces
    var pt = points[i];
    var pf = pointForces(pt, _charge[i], points, _charge);
    var bf = polygonForces(pt, poly, parallelForces);
    var force = [
      Math.pow(10, baseForce) * (pf[0] + (Math.abs(_charge[i]) * wallForces * N * bf[0]) / S),
      Math.pow(10, baseForce) * (pf[1] + (Math.abs(_charge[i]) * wallForces * N * bf[1]) / S),
      // Polygon forces are normalized to it's total length and
      // multiplied by wallForces
    ];
    // Update momentum
    var mx = force[0] / _mass[i] + m[i][0];
    var my = force[1] / _mass[i] + m[i][1];
    var norm = Math.sqrt(Math.pow(mx, 2) + Math.pow(my, 2));
    var mf = norm * (1 - drag);
    mf =
      mf < maxMomentum ? mf : maxMomentum * (Math.exp(-viscosity * (mf - maxMomentum)) * viscosity + (1 - viscosity));
    m[i] = [(mf * mx) / norm, (mf * my) / norm];
    // Update position
    for (var j = 0; j < 10; j++) {
      // We move points as close to the polygon as possible
      var px = pt[0] + m[i][0];
      var py = pt[1] + m[i][1];
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
  return [p, m];
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
export function pointForces(pt, c, points, charge) {
  var f = [0, 0];
  for (var i = 0; i < points.length; i++) {
    var vdx = pt[0] - points[i][0];
    var vdy = pt[1] - points[i][1];
    var norm2 = Math.pow(vdx, 2) + Math.pow(vdy, 2);
    norm2 = norm2 != 0 ? norm2 : Infinity;
    var norm = Math.sqrt(norm2);
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
export function polygonForces(pt, polygon, parallelForces) {
  if (parallelForces === void 0) {
    parallelForces = PARALLELFORCES;
  }
  var f = [0, 0];
  for (var i = 1; i < polygon.length; i++) {
    // Useful vectors
    var a0 = polygon[i - 1]; // A from origin
    var b0 = polygon[i]; // B from origin
    var t = perpendicularToLine(pt, a0, b0);
    var y2 = Math.pow(t[0], 2) + Math.pow(t[1], 2);
    var y = Math.sqrt(y2);
    var normal = [b0[0] - a0[0], b0[1] - a0[1]];
    var n2 = Math.sqrt(Math.pow(normal[0], 2) + Math.pow(normal[1], 2));
    var xn = [normal[0] / n2, normal[1] / n2];
    var yn = [xn[1], -xn[0]];
    var apt = [a0[0] - pt[0], a0[1] - pt[1]];
    var bpt = [b0[0] - pt[0], b0[1] - pt[1]];
    // The Magic
    var thetaT = Math.atan2(t[1], t[0]);
    var thetaA = thetaT - Math.atan2(apt[1], apt[0]);
    var thetaB = thetaT - Math.atan2(bpt[1], bpt[0]);
    var thetaDiff = thetaB - thetaA; // Align thetaDiff between 0 and +PI
    thetaDiff = thetaDiff <= 0 ? thetaDiff + 2 * Math.PI : thetaDiff;
    thetaDiff = thetaDiff > Math.PI ? thetaDiff - 2 * Math.PI : thetaDiff;
    var Ex = 0; // Parallel fraction
    var Ey = 0; // Perpendicular fraction
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
export function perpendicularToLine(pt, v1, v2) {
  var v1v2 = [v1[0] - v2[0], v1[1] - v2[1]];
  var nv1v2 = Math.sqrt(Math.pow(v1v2[0], 2) + Math.pow(v1v2[1], 2));
  var v1p = [v1[0] - pt[0], v1[1] - pt[1]];
  var n = [v1v2[0] / nv1v2, v1v2[1] / nv1v2];
  var d = v1p[0] * n[0] + v1p[1] * n[1];
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
export function checkInbounds(pt, polygon) {
  if (polygon.length < 3) {
    throw new RangeError('Polygon must have at least 3 vertices');
  }
  var n = polygon.length;
  var newPoints = polygon.slice(0);
  newPoints.push(polygon[0]);
  var wn = 0; // Wn counter
  // loop through all edges of the polygon
  for (var i = 0; i < n; i++) {
    if (newPoints[i][1] <= pt[1]) {
      if (newPoints[i + 1][1] > pt[1]) {
        if (isLeft(newPoints[i], newPoints[i + 1], pt) > 0) {
          wn++;
        }
      }
    } else if (newPoints[i + 1][1] <= pt[1]) {
      if (isLeft(newPoints[i], newPoints[i + 1], pt) < 0) {
        wn--;
      }
    }
  }
  // The pt is outside only when this winding number wn===0, otherwise it's inside
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
export function randomInPolygon(N, polygon) {
  var points = [];
  var _a = polygon.reduce(
      function (acc, v) {
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
    ),
    xMin = _a.xMin,
    xMax = _a.xMax,
    yMin = _a.yMin,
    yMax = _a.yMax;
  var deltaX = xMax - xMin;
  var deltaY = yMax - yMin;
  while (points.length < N) {
    var pt = [xMin + Math.random() * deltaX, yMin + Math.random() * deltaY];
    if (checkInbounds(pt, polygon)) {
      points.push(pt);
    }
  }
  return points;
}
/**
 * Creates N points inside a geo polygon,
 *
 * @param {Number} N Number of points
 * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
 *
 * @return {Array} points N points inside the geo polygon
 */
export function randomInGeoPolygon(N, geoPolygon) {
  return randomInPolygon(
    N,
    geoPolygon.map(function (p) {
      return [GEO_MILIMITER_PRECISION * p.lat, GEO_MILIMITER_PRECISION * p.lng];
    })
  ).map(function (p) {
    return {
      lat: p[0] / GEO_MILIMITER_PRECISION,
      lng: p[1] / GEO_MILIMITER_PRECISION,
    };
  });
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
export function relaxPoints(_a) {
  var _b;
  var points = _a.points,
    momentum = _a.momentum,
    polygon = _a.polygon,
    iterations = _a.iterations,
    callback = _a.callback,
    _c = _a.mass,
    mass = _c === void 0 ? DEFAULTMASS : _c,
    _d = _a.charge,
    charge = _d === void 0 ? DEFAULTCHARGE : _d,
    _e = _a.baseForce,
    baseForce = _e === void 0 ? BASEFORCE : _e,
    _f = _a.drag,
    drag = _f === void 0 ? DRAG : _f,
    _g = _a.viscosity,
    viscosity = _g === void 0 ? VISCOSITY : _g,
    _h = _a.maxMomentum,
    maxMomentum = _h === void 0 ? MAXMOMENTUM : _h,
    _j = _a.parallelForces,
    parallelForces = _j === void 0 ? PARALLELFORCES : _j,
    _k = _a.wallForces,
    wallForces = _k === void 0 ? WALLFORCES : _k,
    _l = _a.simplifyPolygon,
    simplifyPolygon = _l === void 0 ? SIMPLIFYPOLYGON : _l,
    _m = _a.attenuation,
    attenuation = _m === void 0 ? ATTENUATION : _m;
  var p = __spreadArray([], points);
  var m =
    momentum == undefined
      ? p.map(function () {
          return [0, 0];
        })
      : __spreadArray([], momentum);
  for (var i = 0; i < iterations; i++) {
    var currentDrag = drag + (1 - drag) * Math.tanh(i * attenuation);
    (_b = movePoints({
      points: p,
      momentum: m,
      polygon: polygon,
      mass: mass,
      baseForce: baseForce,
      drag: currentDrag,
      viscosity: viscosity,
      maxMomentum: maxMomentum,
      parallelForces: parallelForces,
      wallForces: wallForces,
      simplifyPolygon: simplifyPolygon,
    })),
      (p = _b[0]),
      (m = _b[1]);
    if (callback != undefined) {
      callback(p, m, polygon, baseForce, currentDrag, viscosity, maxMomentum);
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
export function relaxNPoints(_a) {
  var N = _a.N,
    polygon = _a.polygon,
    iterations = _a.iterations,
    callback = _a.callback,
    _b = _a.mass,
    mass = _b === void 0 ? DEFAULTMASS : _b,
    _c = _a.charge,
    charge = _c === void 0 ? DEFAULTCHARGE : _c,
    _d = _a.baseForce,
    baseForce = _d === void 0 ? BASEFORCE : _d,
    _e = _a.drag,
    drag = _e === void 0 ? DRAG : _e,
    _f = _a.viscosity,
    viscosity = _f === void 0 ? VISCOSITY : _f,
    _g = _a.maxMomentum,
    maxMomentum = _g === void 0 ? MAXMOMENTUM : _g,
    _h = _a.parallelForces,
    parallelForces = _h === void 0 ? PARALLELFORCES : _h,
    _j = _a.wallForces,
    wallForces = _j === void 0 ? WALLFORCES : _j,
    _k = _a.simplifyPolygon,
    simplifyPolygon = _k === void 0 ? SIMPLIFYPOLYGON : _k,
    _l = _a.attenuation,
    attenuation = _l === void 0 ? ATTENUATION : _l;
  var points = randomInPolygon(N, polygon);
  var momentum = points.map(function () {
    return [0, 0];
  });
  return relaxPoints({
    points: points,
    momentum: momentum,
    polygon: polygon,
    iterations: iterations,
    callback: callback,
    mass: mass,
    baseForce: baseForce,
    drag: drag,
    viscosity: viscosity,
    maxMomentum: maxMomentum,
    parallelForces: parallelForces,
    wallForces: wallForces,
    simplifyPolygon: simplifyPolygon,
    attenuation: attenuation,
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
export function relaxGeoPoints(_a) {
  var geoPoints = _a.geoPoints,
    geoPolygon = _a.geoPolygon,
    width = _a.width,
    iterations = _a.iterations,
    callback = _a.callback,
    _b = _a.mass,
    mass = _b === void 0 ? DEFAULTMASS : _b,
    _c = _a.charge,
    charge = _c === void 0 ? DEFAULTCHARGE : _c,
    _d = _a.baseForce,
    baseForce = _d === void 0 ? BASEFORCE : _d,
    _e = _a.drag,
    drag = _e === void 0 ? DRAG : _e,
    _f = _a.viscosity,
    viscosity = _f === void 0 ? VISCOSITY : _f,
    _g = _a.maxMomentum,
    maxMomentum = _g === void 0 ? MAXMOMENTUM : _g,
    _h = _a.parallelForces,
    parallelForces = _h === void 0 ? PARALLELFORCES : _h,
    _j = _a.wallForces,
    wallForces = _j === void 0 ? WALLFORCES : _j,
    _k = _a.simplifyPolygon,
    simplifyPolygon = _k === void 0 ? SIMPLIFYPOLYGON : _k,
    _l = _a.attenuation,
    attenuation = _l === void 0 ? ATTENUATION : _l;
  var _m = buildPolygon(geoPolygon, width),
    polygon = _m.polygon,
    minLat = _m.minLat,
    minLng = _m.minLng,
    delta = _m.delta;
  var momentum = geoPoints.map(function () {
    return [0, 0];
  });
  var points = relaxPoints({
    points: geoPoints.map(function (p) {
      return [(p.lat - minLat) * delta, (p.lng - minLng) * delta];
    }),
    momentum: momentum,
    polygon: polygon,
    iterations: iterations,
    callback: callback,
    mass: mass,
    baseForce: baseForce,
    drag: drag,
    viscosity: viscosity,
    maxMomentum: maxMomentum,
    parallelForces: parallelForces,
    wallForces: wallForces,
    simplifyPolygon: simplifyPolygon,
    attenuation: attenuation,
  });
  return {
    polygon: polygon,
    points: points,
    geoPoints: points.map(function (pt) {
      return {
        lat: pt[0] / delta + minLat,
        lng: pt[1] / delta + minLng,
      };
    }),
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
export function relaxNGeoPoints(_a) {
  var N = _a.N,
    geoPolygon = _a.geoPolygon,
    width = _a.width,
    iterations = _a.iterations,
    callback = _a.callback,
    _b = _a.mass,
    mass = _b === void 0 ? DEFAULTMASS : _b,
    _c = _a.charge,
    charge = _c === void 0 ? DEFAULTCHARGE : _c,
    _d = _a.baseForce,
    baseForce = _d === void 0 ? BASEFORCE : _d,
    _e = _a.drag,
    drag = _e === void 0 ? DRAG : _e,
    _f = _a.viscosity,
    viscosity = _f === void 0 ? VISCOSITY : _f,
    _g = _a.maxMomentum,
    maxMomentum = _g === void 0 ? MAXMOMENTUM : _g,
    _h = _a.parallelForces,
    parallelForces = _h === void 0 ? PARALLELFORCES : _h,
    _j = _a.wallForces,
    wallForces = _j === void 0 ? WALLFORCES : _j,
    _k = _a.simplifyPolygon,
    simplifyPolygon = _k === void 0 ? SIMPLIFYPOLYGON : _k,
    _l = _a.attenuation,
    attenuation = _l === void 0 ? ATTENUATION : _l;
  var geoPoints = randomInGeoPolygon(N, geoPolygon);
  return relaxGeoPoints({
    geoPoints: geoPoints,
    geoPolygon: geoPolygon,
    width: width,
    iterations: iterations,
    mass: mass,
    callback: callback,
    baseForce: baseForce,
    drag: drag,
    viscosity: viscosity,
    maxMomentum: maxMomentum,
    parallelForces: parallelForces,
    wallForces: wallForces,
    simplifyPolygon: simplifyPolygon,
    attenuation: attenuation,
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
export function buildPolygon(geoPolygon, width) {
  var _a = geoPolygon.reduce(
      function (acc, v) {
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
    ),
    minLat = _a.minLat,
    maxLat = _a.maxLat,
    minLng = _a.minLng,
    maxLng = _a.maxLng;
  var delta = width / (maxLng - minLng);
  var polygon = geoPolygon.map(function (v) {
    return [delta * (v.lat - minLat), delta * (v.lng - minLng)];
  });
  if (polygon[0][0] != polygon[polygon.length - 1][0] || polygon[0][1] != polygon[polygon.length - 1][1]) {
    polygon.push(polygon[0]);
  }
  return { polygon: polygon, minLat: minLat, minLng: minLng, delta: delta };
}
