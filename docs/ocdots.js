/*
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
var __spreadArray = (this && this.__spreadArray) || function (to, from) {
    for (var i = 0, il = from.length, j = to.length; i < il; i++, j++)
        to[j] = from[i];
    return to;
};
// Default values
export var BASEFORCE = 5;
export var DRAG = 0.1;
export var VISCOSITY = 0.1;
export var MAXMOMENTUM = 5;
export var PARALLELFORCES = true;
export var ATTENUATION = 0.01;
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
export function movePoints(_a) {
    var points = _a.points, momentum = _a.momentum, polygon = _a.polygon, _b = _a.baseForce, baseForce = _b === void 0 ? BASEFORCE : _b, _c = _a.drag, drag = _c === void 0 ? DRAG : _c, _d = _a.viscosity, viscosity = _d === void 0 ? VISCOSITY : _d, _e = _a.maxMomentum, maxMomentum = _e === void 0 ? MAXMOMENTUM : _e, _f = _a.parallelForces, parallelForces = _f === void 0 ? PARALLELFORCES : _f;
    var p = __spreadArray([], points);
    var m = __spreadArray([], momentum);
    var N = points.length;
    if (polygon[0][0] != polygon[polygon.length - 1][0] ||
        polygon[0][1] != polygon[polygon.length - 1][1]) {
        polygon.push(polygon[0]);
    }
    var S = polygon.reduce(function (acc, cur) {
        return acc + Math.sqrt(Math.pow(cur[0], 2) + Math.pow(cur[1], 2));
    }, 0);
    // Calculate forces
    var pf = p.map(function (pt) { return pointForces(pt, p); });
    var bf = p.map(function (pt) { return polygonForces(pt, polygon, parallelForces); });
    // Update momentum
    m = m.map(function (mt, i) {
        var force = [
            Math.pow(10, baseForce) * (pf[i][0] + (2 * N * bf[i][0]) / S),
            Math.pow(10, baseForce) * (pf[i][1] + (2 * N * bf[i][1]) / S),
            // Polygon forces are normalized to it's total length and
            // multiplied by 2N as if there's 2 charges in the walls
            // for each point
        ];
        return updateMomentum(mt, force, drag, viscosity, maxMomentum);
    });
    // Update position
    p = p.map(function (pt, i) {
        var px = pt[0] + m[i][0];
        var py = pt[1] + m[i][1];
        if (!checkInbounds([px, py], polygon)) {
            m[i][0] = 0;
            m[i][1] = 0;
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
export function pointForces(pt, points) {
    return points.reduce(function (acc, pt0) {
        var vdx = pt[0] - pt0[0];
        var vdy = pt[1] - pt0[1];
        var norm2 = Math.pow(vdx, 2) + Math.pow(vdy, 2);
        norm2 = norm2 != 0 ? norm2 : Infinity;
        var norm = Math.sqrt(norm2);
        var fx = vdx / norm / norm2;
        var fy = vdy / norm / norm2;
        return [acc[0] + fx, acc[1] + fy];
    }, [0, 0]);
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
export function polygonForces(pt, polygon, parallelForces) {
    if (parallelForces === void 0) { parallelForces = PARALLELFORCES; }
    return polygon.slice(1).reduce(function (acc, v2, i1) {
        var v1 = polygon[i1];
        var t = perpendicularToLine(pt, v1, v2);
        var p = [-t[1], t[0]];
        var nt = Math.sqrt(Math.pow(t[0], 2) + Math.pow(t[1], 2));
        var v1pt = [pt[0] - v1[0], pt[1] - v1[1]];
        var v2pt = [pt[0] - v2[0], pt[1] - v2[1]];
        var thetaA = Math.atan2(v1pt[1], v1pt[0]) - Math.atan2(t[1], t[0]);
        var thetaB = Math.atan2(v2pt[1], v2pt[0]) - Math.atan2(t[1], t[0]);
        // Align thetaDiff between -PI and +PI
        var thetaDiff = thetaB - thetaA;
        thetaDiff = thetaDiff <= Math.PI ? thetaDiff + 2 * Math.PI : thetaDiff;
        thetaDiff = thetaDiff > Math.PI ? thetaDiff - 2 * Math.PI : thetaDiff;
        var modulus = (1 / nt) * Math.sin((1 / 2) * thetaDiff);
        var tv = Math.sin(thetaB) - Math.sin(thetaA); // Perpendicular fraction
        var pv = -(Math.cos(thetaB) - Math.cos(thetaA)); // Parallel fraction
        var dn = Math.sqrt(Math.pow(pv, 2) + Math.pow(tv, 2));
        var direction = parallelForces
            ? [
                (tv * t[0]) / nt / dn + (pv * p[0]) / nt / dn,
                (tv * t[1]) / nt / dn + (pv * p[1]) / nt / dn,
            ]
            : [(tv * t[0]) / nt / dn, (tv * t[1]) / nt / dn];
        var f = [modulus * direction[0], modulus * direction[1]];
        return [acc[0] + f[0], acc[1] + f[1]];
    }, [0, 0]);
}
/*
 * Returns the perpendicular line to segment v1 -> v2.
 *
 * @param {Array} pt Point to calculate perpendicular to
 * @param {Array} v1 Line segment vertetx 1
 * @param {Array} v2 Line segment vertetx 2
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
export function updateMomentum(mt, force, drag, viscosity, maxMomentum) {
    var mx = force[0] + mt[0];
    var my = force[1] + mt[1];
    var norm2 = Math.pow(mx, 2) + Math.pow(my, 2);
    var norm = Math.sqrt(norm2);
    var intensity = norm - drag * norm2;
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
export function checkInbounds(pt, polygon) {
    var inbound = false;
    var x = pt[0], y = pt[1];
    for (var i = 1; i < polygon.length; i++) {
        var _a = polygon[i - 1], p1x = _a[0], p1y = _a[1];
        var _b = polygon[i], p2x = _b[0], p2y = _b[1];
        if (p2y > y != p1y > y && x < p2x + ((p1x - p2x) * (y - p2y)) / (p1y - p2y))
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
export function randomInPolygon(N, polygon) {
    var points = [];
    var _a = polygon.reduce(function (acc, v) {
        acc.xMin = v[0] < acc.xMin ? v[0] : acc.xMin;
        acc.xMax = v[0] > acc.xMax ? v[0] : acc.xMax;
        acc.yMin = v[1] < acc.yMin ? v[1] : acc.yMin;
        acc.yMax = v[1] > acc.yMax ? v[1] : acc.yMax;
        return acc;
    }, {
        xMin: Infinity,
        xMax: -Infinity,
        yMin: Infinity,
        yMax: -Infinity,
    }), xMin = _a.xMin, xMax = _a.xMax, yMin = _a.yMin, yMax = _a.yMax;
    var deltaX = xMax - xMin;
    var deltaY = yMax - yMin;
    while (points.length < N) {
        var pt = [
            xMin + Math.random() * deltaX,
            yMin + Math.random() * deltaY,
        ];
        if (checkInbounds(pt, polygon))
            points.push(pt);
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
export function randomInGeoPolygon(N, geoPolygon) {
    return randomInPolygon(N, geoPolygon.map(function (p) { return [p.lat, p.lng]; })).map(function (p) { return ({ lat: p[0], lng: p[1] }); });
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
export function relaxPoints(_a) {
    var _b;
    var points = _a.points, momentum = _a.momentum, polygon = _a.polygon, iterations = _a.iterations, callback = _a.callback, _c = _a.baseForce, baseForce = _c === void 0 ? BASEFORCE : _c, _d = _a.drag, drag = _d === void 0 ? DRAG : _d, _e = _a.viscosity, viscosity = _e === void 0 ? VISCOSITY : _e, _f = _a.maxMomentum, maxMomentum = _f === void 0 ? MAXMOMENTUM : _f, _g = _a.parallelForces, parallelForces = _g === void 0 ? PARALLELFORCES : _g, _h = _a.attenuation, attenuation = _h === void 0 ? ATTENUATION : _h;
    var p = __spreadArray([], points);
    var m = momentum == undefined ? p.map(function () { return [0, 0]; }) : __spreadArray([], momentum);
    var att = 1 + attenuation;
    for (var i = 0, attIter = att; i < iterations; i++, attIter *= att) {
        _b = movePoints({
            points: p,
            momentum: m,
            polygon: polygon,
            baseForce: baseForce,
            drag: drag * attIter,
            viscosity: viscosity,
            maxMomentum: maxMomentum,
            parallelForces: parallelForces,
        }), p = _b[0], m = _b[1];
        if (callback != undefined) {
            callback(p, m, polygon, baseForce, drag * attIter, viscosity, maxMomentum);
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
export function relaxNPoints(_a) {
    var N = _a.N, polygon = _a.polygon, iterations = _a.iterations, callback = _a.callback, _b = _a.baseForce, baseForce = _b === void 0 ? BASEFORCE : _b, _c = _a.drag, drag = _c === void 0 ? DRAG : _c, _d = _a.viscosity, viscosity = _d === void 0 ? VISCOSITY : _d, _e = _a.maxMomentum, maxMomentum = _e === void 0 ? MAXMOMENTUM : _e, _f = _a.parallelForces, parallelForces = _f === void 0 ? PARALLELFORCES : _f, _g = _a.attenuation, attenuation = _g === void 0 ? ATTENUATION : _g;
    var points = randomInPolygon(N, polygon);
    var momentum = points.map(function () { return [0, 0]; });
    return relaxPoints({
        points: points,
        momentum: momentum,
        polygon: polygon,
        iterations: iterations,
        callback: callback,
        baseForce: baseForce,
        drag: drag,
        viscosity: viscosity,
        maxMomentum: maxMomentum,
        parallelForces: parallelForces,
        attenuation: attenuation,
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
export function relaxGeoPoints(_a) {
    var geoPoints = _a.geoPoints, geoPolygon = _a.geoPolygon, width = _a.width, iterations = _a.iterations, callback = _a.callback, _b = _a.baseForce, baseForce = _b === void 0 ? BASEFORCE : _b, _c = _a.drag, drag = _c === void 0 ? DRAG : _c, _d = _a.viscosity, viscosity = _d === void 0 ? VISCOSITY : _d, _e = _a.maxMomentum, maxMomentum = _e === void 0 ? MAXMOMENTUM : _e, _f = _a.parallelForces, parallelForces = _f === void 0 ? PARALLELFORCES : _f, _g = _a.attenuation, attenuation = _g === void 0 ? ATTENUATION : _g;
    var _h = buildPolygon(geoPolygon, width), polygon = _h.polygon, minLat = _h.minLat, minLng = _h.minLng, delta = _h.delta;
    var momentum = geoPoints.map(function () { return [0, 0]; });
    var points = relaxPoints({
        points: geoPoints.map(function (p) { return [
            (p.lat - minLat) * delta,
            (p.lng - minLng) * delta,
        ]; }),
        momentum: momentum,
        polygon: polygon,
        iterations: iterations,
        callback: callback,
        baseForce: baseForce,
        drag: drag,
        viscosity: viscosity,
        maxMomentum: maxMomentum,
        parallelForces: parallelForces,
        attenuation: attenuation,
    });
    return {
        polygon: polygon,
        points: points,
        geoPoints: points.map(function (pt) { return ({
            lat: pt[0] / delta + minLat,
            lng: pt[1] / delta + minLng,
        }); }),
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
export function relaxNGeoPoints(_a) {
    var N = _a.N, geoPolygon = _a.geoPolygon, width = _a.width, iterations = _a.iterations, callback = _a.callback, _b = _a.baseForce, baseForce = _b === void 0 ? BASEFORCE : _b, _c = _a.drag, drag = _c === void 0 ? DRAG : _c, _d = _a.viscosity, viscosity = _d === void 0 ? VISCOSITY : _d, _e = _a.maxMomentum, maxMomentum = _e === void 0 ? MAXMOMENTUM : _e, _f = _a.parallelForces, parallelForces = _f === void 0 ? PARALLELFORCES : _f, _g = _a.attenuation, attenuation = _g === void 0 ? ATTENUATION : _g;
    var geoPoints = randomInGeoPolygon(N, geoPolygon);
    return relaxGeoPoints({
        geoPoints: geoPoints,
        geoPolygon: geoPolygon,
        width: width,
        iterations: iterations,
        callback: callback,
        baseForce: baseForce,
        drag: drag,
        viscosity: viscosity,
        maxMomentum: maxMomentum,
        parallelForces: parallelForces,
        attenuation: attenuation,
    });
}
/*
 * Transforms a set of coordinates into a polygon with a known width
 *
 * @param {Array} geoPolygon Polygon of geo coordinates {lat, lng}
 * @param {Number} width Width of the polygon
 * @return {Object} { polygon, minLat, minLng, delta }
 */
export function buildPolygon(geoPolygon, width) {
    var _a = geoPolygon.reduce(function (acc, v) {
        acc.minLat = v.lat < acc.minLat ? v.lat : acc.minLat;
        acc.maxLat = v.lat > acc.maxLat ? v.lat : acc.maxLat;
        acc.minLng = v.lng < acc.minLng ? v.lng : acc.minLng;
        acc.maxLng = v.lng > acc.maxLng ? v.lng : acc.maxLng;
        return acc;
    }, {
        minLat: Infinity,
        maxLat: -Infinity,
        minLng: Infinity,
        maxLng: -Infinity,
    }), minLat = _a.minLat, maxLat = _a.maxLat, minLng = _a.minLng, maxLng = _a.maxLng;
    var delta = width / (maxLng - minLng);
    var polygon = geoPolygon.map(function (v) { return [delta * (v.lat - minLat), delta * (v.lng - minLng)]; });
    // polygon = sortPolygon(polygon);
    if (polygon[0][0] != polygon[polygon.length - 1][0] ||
        polygon[0][1] != polygon[polygon.length - 1][1]) {
        polygon.push(polygon[0]);
    }
    return { polygon: polygon, minLat: minLat, minLng: minLng, delta: delta };
}
/*
 * Sorts the polygon vertexes.
 *
 * @param {Array} polygon Set of points that describes the polygon
 * @return {Array} polygon Sorted polygon
 */
export function sortPolygon(polygon) {
    var centroid = polygon.reduce(function (acc, v, i, arr) { return [
        acc[0] + v[0] / arr.length,
        acc[1] + v[1] / arr.length,
    ]; }, [0, 0]);
    polygon = polygon.sort(function (a, b) {
        return Math.atan2(b[1] - centroid[1], b[0] - centroid[0]) -
            Math.atan2(a[1] - centroid[1], a[0] - centroid[0]);
    });
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
export function drawPolygon(ctx, polygon, color) {
    if (color === void 0) { color = "black"; }
    ctx.strokestyle = color;
    var pt0 = polygon[0];
    ctx.beginPath();
    ctx.moveTo(pt0[0], pt0[1]);
    polygon.slice(1).forEach(function (pt) {
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
export function drawPoints(ctx, points, radius, color) {
    if (radius === void 0) { radius = 6; }
    if (color === void 0) { color = "black"; }
    ctx.fillStyle = color;
    points.forEach(function (pt) {
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
export function resetCanvas(canvas, backgroundColor) {
    if (backgroundColor === void 0) { backgroundColor = "white"; }
    var ctx = canvas.getContext("2d");
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
export function drawPolygonAndPoints(canvas, points, polygon, radius, color, backgroundColor) {
    if (radius === void 0) { radius = 6; }
    if (color === void 0) { color = "black"; }
    if (backgroundColor === void 0) { backgroundColor = "white"; }
    var ctx = canvas.getContext("2d");
    var _a = polygon.reduce(function (acc, v) {
        acc.width = acc.width < v[0] ? v[0] : acc.width;
        acc.height = acc.height < v[1] ? v[1] : acc.height;
        return acc;
    }, { width: 0, height: 0 }), width = _a.width, height = _a.height;
    resetCanvas(canvas, backgroundColor);
    drawPoints(ctx, points, radius, color);
    drawPolygon(ctx, polygon, color);
}
