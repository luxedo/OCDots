/*
 * OCDots
 * Copyright (C) 2020 Luiz Eduardo Amaral <luizamaral306@gmail.com>
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
const SIMPLIFYPOLYGON = 20;
const BASEFORCE = 5;
const DRAG = 0.1;
const VISCOSITY = 0.1;
const MAXMOMENTUM = 10;
const PARALLELFORCES = true;
const ATTENUATION = 0.01;

const ocdots = (() => {
  /*
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
   * @param {Array} points The points to move
   * @param {Array} momentum Accumulated momentum for each point. [0, 0]
   *      when the points are stopped.
   * @param {Array} polygon Set of points that describes the polygon that
   *      contains the points. This polygon should be ordered (clockwise
   *      or anticlockwise) and closed i.e. first points equals the last
   *      point.
   * @param {Number} baseForce The force constant
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @return {Array, Array} points,momentum Updated points and momentum
   *      arrays
   */
  function movePoints({
    points,
    momentum,
    polygon,
    baseForce = BASEFORCE,
    drag = DRAG,
    viscosity = VISCOSITY,
    maxMomentum = MAXMOMENTUM,
    parallelForces = PARALLELFORCES,
  }) {
    let p = [...points];
    let m = [...momentum];
    const N = points.length;
    const S = polygon.reduce((acc, cur) => {
      return acc + Math.sqrt(Math.pow(cur[0], 2) + Math.pow(cur[1], 2));
    }, 0);

    // Calculate forces
    const pf = p.map((pt) => pointForces(pt, p));
    const bf = p.map((pt) => polygonForces(pt, polygon, parallelForces));

    // Update momentum
    m = m.map((mt, i) => {
      const force = [
        Math.pow(10, baseForce) * (pf[i][0] + (2 * N * bf[i][0]) / S),
        Math.pow(10, baseForce) * (pf[i][1] + (2 * N * bf[i][1]) / S),
        // Polygon forces are normalized to it's total length and
        // multiplied by 2N as if there's 2 charges in the walls
        // for each point
      ];
      return updateMomentum(mt, force, drag, viscosity, maxMomentum);
    });

    // Update position
    p = p.map((pt, i) => {
      const px = pt[0] + m[i][0];
      const py = pt[1] + m[i][1];
      if (!checkInbounds([px, py], polygon)) {
        m[i][0] = 0;
        m[i][1] = 1;
        return pt;
      }
      return [px, py];
    });
    return [p, m];
  }

  /*
   * Calculates forces on pt from points.
   *
   * @param {Array} pt The point to measure forces
   * @param {Array} points The points acting on pt
   * @return {Array} force Sum of forces acting on pt
   */
  function pointForces(pt, p) {
    return p.reduce(
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

  /*
   * Calculates forces on pt from polygon. The forces are the integral
   * of the forces for each line segment for a charged wire as in:
   * https://aapt.scitation.org/doi/full/10.1119/1.4906421
   *
   * The default behavior is to sum perpendicular and parallel forces
   * for each line segment, setting `parallelForces` to `false` sums
   * only perpendicular forces, but this may lead to instability
   * because of nonlinearity in the forces for concave polygons.
   *
   * @param {Array} pt The point to measure forces
   * @param {Array} polygon The polygons vertexes
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @return {Array} force Sum of forces acting on pt
   */
  function polygonForces(pt, polygon, parallelForces = PARALLELFORCES) {
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

  /*
   * Returns the perpendicular line to segment v1 -> v2.
   *
   * @param {Array} pt Point to calculate perpendicular to
   * @param {Array} v1 Line segment vertetx 1
   * @param {Array} v2 Line segment vertetx 2
   * @return {Array} p Perpendicular vector
   */
  function perpendicularToLine(pt, v1, v2) {
    const v1v2 = [v1[0] - v2[0], v1[1] - v2[1]];
    const nv1v2 = Math.sqrt(Math.pow(v1v2[0], 2) + Math.pow(v1v2[1], 2));
    const v1p = [v1[0] - pt[0], v1[1] - pt[1]];
    const n = [v1v2[0] / nv1v2, v1v2[1] / nv1v2];
    const d = v1p[0] * n[0] + v1p[1] * n[1];
    return [v1p[0] - d * n[0], v1p[1] - d * n[1]];
  }

  /*
   * Updates the momentum.
   *
   * @param {Array} mt Initial momentum
   * @param {Array} force Forcing acting at the point
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @return {Array} mu Updated momentum
   */
  function updateMomentum(mt, force, drag, viscosity, maxMomentum) {
    const mx = force[0] + mt[0];
    const my = force[1] + mt[1];
    const norm2 = Math.pow(mx, 2) + Math.pow(my, 2);
    const norm = Math.sqrt(norm2);
    let intensity = norm - drag * norm2;
    intensity = intensity > maxMomentum ? maxMomentum : intensity;
    intensity =
      intensity < 0
        ? viscosity == 0
          ? maxMomentum
          : maxMomentum * Math.exp(-viscosity) // High forces viscosity
        : intensity;
    return [(intensity * mx) / norm, (intensity * my) / norm];
  }

  /*
   * Checks if the point pt is inside polygon.
   *
   * @param {Array} pt The point to measure forces
   * @param {Array} polygon Set of points that describes the polygon.
   *      This polygon should be ordered (clockwise or anticlockwise) and
   *      closed i.e. first points equals the last point.
   * @return {boolean} inbound True if pt is inside the polygon
   */
  function checkInbounds(pt, polygon) {
    let inbound = false;
    const [x, y] = pt;
    for (let i = 1; i < polygon.length; i++) {
      const [p1x, p1y] = polygon[i - 1];
      const [p2x, p2y] = polygon[i];
      if (
        p2y > y != p1y > y &&
        x < p2x + ((p1x - p2x) * (y - p2y)) / (p1y - p2y)
      )
        inbound = !inbound;
    }
    return inbound;
  }

  /*
   * Creates N points inside the polygon
   *
   * @param {Number} N Number of points
   * @param {Array} polygon Set of points that describes the polygon.
   *      This polygon should be ordered (clockwise or anticlockwise) and
   *      closed i.e. first points equals the last point.
   * @return {Array} points N points inside the polygon
   */
  function randomInPolygon(N, polygon) {
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
        xMax: 0,
        yMin: Infinity,
        yMax: 0,
      }
    );
    const deltaX = xMax - xMin;
    const deltaY = yMax - yMin;
    while (points.length < N) {
      const pt = [xMin + Math.random() * deltaX, yMin + Math.random() * deltaY];
      if (checkInbounds(pt, polygon)) points.push(pt);
    }
    return points;
  }

  /*
   * Creates N points inside a geo polygon,
   *
   * @param {Number} N Number of points
   * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
   * @return {Array} points N points inside the geo polygon
   */
  function randomInGeoPolygon(N, geoPolygon) {
    return randomInPolygon(
      N,
      geoPolygon.map((p) => [p.lat, p.lng])
    ).map((p) => ({ lat: p[0], lng: p[1] }));
  }

  /*
   * Runs several iterations of movePoints(). The drag increases in every
   * iteration attenuating the movement.
   *
   * @param {Array} points The points to move
   * @param {Array} momentum Initial momentum. [0, 0] for all points if
   *      ommited
   * @param {Array} polygon Set of points that describes the polygon that
   *      contains the points. This polygon should be ordered (clockwise
   *      or anticlockwise) and closed i.e. first points equals the last
   *      point.
   * @param {Number} iterations Number of iterations to run
   * @param {Number} attenuation Rate of attenuation
   * @param {Number} baseForce The force constant
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @param {Function} callback Callback function to run at every
   *      iteration (optional). Callback args: points, momentum, polygon,
   *      baseForce, currentDrag, viscosity, maxMomentum
   * @return {Array} points Last iteration points positions
   */
  function relaxPoints({
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
    attenuation = ATTENUATION,
  }) {
    let p = [...points];
    let m = momentum == undefined ? p.map(() => [0, 0]) : [...momentum];

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

  /*
   * Calls relaxPoints for N random points placed inside the polygon.
   *
   * @param {Number} N Number of points
   * @param {Array} polygon Set of points that describes the polygon that
   *      contains the points. This polygon should be ordered (clockwise
   *      or anticlockwise) and closed i.e. first points equals the last
   *      point.
   * @param {Number} iterations Number of iterations to run
   * @param {Function} callback Callback function to run at every
   * @param {Number} baseForce The force constant
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @param {Number} attenuation Rate of attenuation
   *      iteration (optional). Callback args: points, momentum, polygon,
   *      baseForce, currentDrag, viscosity, maxMomentum
   * @return {Array} points Last iteration points positions
   */

  function relaxNPoints({
    N,
    polygon,
    iterations,
    callback,
    baseForce = BASEFORCE,
    drag = DRAG,
    viscosity = VISCOSITY,
    maxMomentum = MAXMOMENTUM,
    parallelForces = PARALLELFORCES,
    attenuation = ATTENUATION,
  }) {
    const points = randomInPolygon(N, polygon);
    return relaxPoints({
      points,
      polygon,
      iterations,
      callback,
      baseForce,
      drag,
      viscosity,
      maxMomentum,
      parallelForces,
      attenuation,
    });
  }

  /*
   * Converts geoPoints and geoPolygon to a points and polygon, then
   * calls relaxPoints, returning the last position of the points.
   *
   * @param {Array} geoPoints Points in geo coordinates lat, lng}
   * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
   * @param {Number} width Width of the polygon
   * @param {Number} iterations Number of iterations to run
   * @param {Function} callback Callback function to run at every
   * @param {Number} simplifyPolygon minimum vertex distances
   * @param {Number} baseForce The force constant
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @param {Number} attenuation Rate of attenuation
   *      iteration. Callback args: points, momentum, polygon, baseForce,
   *      currentDrag, viscosity, maxMomentum
   * @return {Object} { polygon, points, geoPoints } Last iteration geo
   *      points positions
   */
  function relaxGeoPoints({
    geoPoints,
    geoPolygon,
    width,
    iterations,
    callback,
    simplifyPolygon = SIMPLIFYPOLYGON,
    baseForce = BASEFORCE,
    drag = DRAG,
    viscosity = VISCOSITY,
    maxMomentum = MAXMOMENTUM,
    parallelForces = PARALLELFORCES,
    attenuation = ATTENUATION,
  }) {
    const { polygon, minLat, minLng, delta } = buildPolygon(
      geoPolygon,
      width,
      simplifyPolygon
    );
    const points = relaxPoints({
      points: geoPoints.map((p) => [
        (p.lat - minLat) * delta,
        (p.lng - minLng) * delta,
      ]),
      polygon,
      iterations,
      callback,
      baseForce,
      drag,
      viscosity,
      maxMomentum,
      parallelForces,
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

  /*
   * Calls relaxGeoPoints for N random points placed inside the polygon.
   *
   * @param {Number} N Number of points
   * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
   * @param {Number} width Width of the polygon
   * @param {Number} iterations Number of iterations to run
   * @param {Function} callback Callback function to run at every
   * @param {Number} simplifyPolygon minimum vertex distances
   * @param {Number} baseForce The force constant
   * @param {Number} drag The drag coeficient
   * @param {Number} viscosity The viscosity coeficient
   * @param {Number} maxMomentum Maximum momentum for each point
   * @param {Boolean} parallelForces Sum line segmen parallel forces
   *    as well.
   * @param {Number} attenuation Rate of attenuation
   *      iteration. Callback args: points, momentum, polygon, baseForce,
   *      currentDrag, viscosity, maxMomentum
   * @return {Object} { polygon, points, geoPoints } Last iteration geo
   *      points positions
   */
  function relaxNGeoPoints({
    N,
    geoPolygon,
    width,
    iterations,
    callback,
    simplifyPolygon = SIMPLIFYPOLYGON,
    baseForce = BASEFORCE,
    drag = DRAG,
    viscosity = VISCOSITY,
    maxMomentum = MAXMOMENTUM,
    parallelForces = PARALLELFORCES,
    attenuation = ATTENUATION,
  }) {
    const geoPoints = randomInGeoPolygon(N, geoPolygon);
    return relaxGeoPoints({
      geoPoints,
      geoPolygon,
      width,
      iterations,
      callback,
      simplifyPolygon,
      baseForce,
      drag,
      viscosity,
      maxMomentum,
      parallelForces,
      attenuation,
    });
  }

  /*
   * Transforms a set of coordinates into a polygon with a known width
   *
   * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
   * @param {Number} width Width of the polygon
   * @param {Number} simplifyPolygon Minimum distance between vertices.
   *      If vertices are closer than this, the one of them is discarded.
   * @return {Object} { polygon, minLat, minLng, delta }
   */
  function buildPolygon(geoPolygon, width, simplifyPolygon) {
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
    let polygon = geoPolygon
      .map((v) => [delta * (v.lat - minLat), delta * (v.lng - minLng)])
      .filter((v1, i1, arr) => {
        return arr.reduce((acc, v2, i2) =>
          i1 == i2
            ? acc
            : !acc
            ? acc
            : Math.sqrt(
                Math.pow(v1[0] - v2[0], 2) + Math.pow(v1[1] - v2[1], 2)
              ) > simplifyPolygon
        );
      }, true);
    polygon = sortPolygon(polygon);
    return { polygon, minLat, minLng, delta };
  }

  /*
   * Sorts the polygon vertexes.
   *
   * @param {Array} polygon Set of points that describes the polygon
   * @return {Array} polygon Sorted polygon
   */
  function sortPolygon(polygon) {
    const centroid = polygon.reduce(
      (acc, v, i, arr) => [
        acc[0] + v[0] / arr.length,
        acc[1] + v[1] / arr.length,
      ],
      [0, 0]
    );
    polyton = polygon.sort(
      (a, b) =>
        Math.atan2(b[1] - centroid[1], b[0] - centroid[0]) -
        Math.atan2(a[1] - centroid[1], a[0] - centroid[0])
    );
    polygon.push(polygon[0]);
    return polygon;
  }

  /*
   * Plots a polygon in the canvas
   *
   * @param {Object} ctx Canvas context
   * @param {Array} polygon Set of points that describes the polygon
   * @param {String} color Points color
   */
  function drawPolygon(ctx, polygon, color = "black") {
    ctx.strokestyle = color;
    pt0 = polygon[0];
    ctx.beginPath();
    ctx.moveTo(pt0[0], pt0[1]);
    polygon.slice(1).forEach((pt) => {
      ctx.lineTo(pt[0], pt[1]);
    });
    ctx.closePath();
    ctx.stroke();
  }

  /*
   * Plots points in the canvas
   *
   * @param {Object} ctx Canvas context
   * @param {Array} polygon Set of points that describes the polygon
   * @param {Number} radius points radius
   * @param {String} color Points color
   */
  function drawPoints(ctx, points, radius = 6, color = "black") {
    ctx.fillStyle = color;
    points.forEach((pt) => {
      ctx.beginPath();
      ctx.arc(pt[0], pt[1], radius, 0, 2 * Math.PI, 0);
      ctx.fill();
    });
  }

  /*
   * Clears the canvas and sets a background color
   *
   * @param {Object} canvas Canvas object
   * @param {String} backgroundColor Points color
   */
  function resetCanvas(canvas, backgroundColor = "white") {
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = backgroundColor;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }

  /*
   * Plots the points and polygon into a blank canvas
   *
   * @param {Object} canvas Canvas object
   * @param {Array} polygon Set of points that describes the polygon
   * @param {Number} radius points radius
   * @param {String} color Points color
   * @param {String} backgroundColor Points color
   */
  function drawPolygonAndPoints(
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

  return {
    movePoints,
    randomInPolygon,
    randomInGeoPolygon,
    relaxPoints,
    relaxNPoints,
    relaxGeoPoints,
    relaxNGeoPoints,
    buildPolygon,
    drawPolygon,
    drawPoints,
    resetCanvas,
    drawPolygonAndPoints,
  };
})();

(function (root, factory) {
  if (typeof define === "function" && define.amd) {
    // AMD. Register as an anonymous module.
    define([], factory);
  } else if (typeof module === "object" && module.exports) {
    // Node.
    module.exports = factory();
  } else {
    // Browser globals (root is window)
    root.returnExports = factory(root.b);
  }
})(typeof self !== "undefined" ? self : this, function () {
  // Use b in some fashion.
  // Just return a value to define the module export.    // This example returns an object, but the module    // can return a function as the exported value.
  return ocdots;
});
