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
const { assert } = require("chai");
const ocdots = require("ocdots");
const { createCanvas } = require("canvas");

const size = 500;
const squarePolygon = [
  [0, 0],
  [0, size],
  [size, size],
  [size, 0],
  [0, 0],
];
const concavePolygon = [
  [0, 0],
  [0, size],
  [size / 2, size],
  [size / 2, size / 2],
  [size, size / 2],
  [size, 0],
  [0, 0],
];
const trianglePolygon = [
  [0, 0],
  [0, size],
  [size, size],
  [0, 0],
];
const squareGeoPolygon = [
  { lat: -45, lng: -45 },
  { lat: -45, lng: 45 },
  { lat: 45, lng: 45 },
  { lat: 45, lng: -45 },
  { lat: -45, lng: -45 },
];

describe("Test public functions", () => {
  const Ntests = 1000;
  it("Creates N points inside polygon", () => {
    for (let i = 0; i < Ntests; i++) {
      const N = 10;
      const polygon = trianglePolygon;
      const lower = Math.atan2(0, 1);
      const upper = Math.atan2(1, 1);

      const p = ocdots.randomInPolygon(N, trianglePolygon);
      const pOffbounds = p.filter(
        ([x, y]) => Math.atan2(y, x) > lower && Math.atan2(y, x) < upper
      );
      assert.equal(p.length, N);
      assert.equal(pOffbounds.length, 0);
    }
  });
  it("Moves points for one iteration", () => {
    for (let i = 0; i < Ntests; i++) {
      const N = 10;
      const polygon = trianglePolygon;
      const points = ocdots.randomInPolygon(N, trianglePolygon);
      const momentum = points.map(() => [0, 0]);
      const maxMomentum = 5;
      const didNotMoved = ocdots
        .movePoints({ points, momentum, polygon, maxMomentum })[0]
        .reduce((acc, pt, idx) => {
          const d = Math.sqrt(
            Math.pow(pt[0] - points[idx][0], 2) +
              Math.pow(pt[1] - points[idx][1], 2)
          );
          assert.isAtMost(Math.sqrt(d), maxMomentum + 0.1);
          if (d == 0) acc++;
          return acc;
        }, 0);
      assert.isAtMost(didNotMoved, 1);
    }
  });
  it("Runs several iterations for points", () => {
    const N = 10;
    const points = ocdots.randomInPolygon(N, squarePolygon);
    const iterations = 900;
    const pf = ocdots.relaxPoints({
      points,
      polygon: squarePolygon,
      iterations,
    });
    pf.forEach((pt1, idx1, arr) => {
      let minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 100);
    });
  });
  it("Runs several iterations for N points", () => {
    const N = 10;
    const iterations = 900;
    const pf = ocdots.relaxNPoints({ N, polygon: squarePolygon, iterations });
    pf.forEach((pt1, idx1, arr) => {
      let minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 100);
    });
  });
  it("Runs several iterations for geo points", () => {
    const N = 10;
    const geoPoints = ocdots.randomInGeoPolygon(N, squareGeoPolygon);
    const iterations = 900;
    const width = 600;
    const pf = ocdots.relaxGeoPoints({
      geoPoints,
      geoPolygon: squareGeoPolygon,
      width,
      iterations,
    });
    pf.points.forEach((pt1, idx1, arr) => {
      const minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 100);
    });
  });
  it("Runs several iterations for N geo points", () => {
    const N = 10;
    const iterations = 900;
    const width = 600;
    const pf = ocdots.relaxNGeoPoints({
      N,
      geoPolygon: squareGeoPolygon,
      width,
      iterations,
    });
    pf.points.forEach((pt1, idx1, arr) => {
      const minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 100);
    });
  });
  it("Runs the callback after every iteration of relaxPoints", () => {
    const N = 10;
    const iterations = 900;
    let cbCounter = 0;
    const callback = (points) => {
      cbCounter++;
    };
    const pf = ocdots.relaxNPoints({
      N,
      polygon: squarePolygon,
      iterations,
      callback,
    });
    pf.forEach((pt1, idx1, arr) => {
      const minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 100);
    });
    assert.equal(cbCounter, iterations);
  });
  it("Runs several iterations for N points for a concave polygon", () => {
    const N = 10;
    const iterations = 900;
    const pf = ocdots.relaxNPoints({ N, polygon: concavePolygon, iterations });
    pf.forEach((pt1, idx1, arr) => {
      const minDist = arr.reduce((acc, pt2, idx2) => {
        const dist = Math.sqrt(
          Math.pow(pt2[0] - pt1[0], 2) + Math.pow(pt2[1] - pt1[1], 2)
        );
        return idx2 != idx1 ? (dist < acc ? dist : acc) : acc;
      }, Infinity);
      assert.isAtLeast(minDist, 50);
    });
  });
  it("Draws points into a canvas", () => {
    const canvas = createCanvas();
    const ctx = canvas.getContext("2d");
    const N = 20;
    const points = ocdots.randomInPolygon(N, squarePolygon);
    ocdots.drawPolygonAndPoints(canvas, points, squarePolygon);
  });
});
