import * as ocdots from "./ocdots.js";
import { drawPolygonAndPoints } from "./draw.js";

document.addEventListener("DOMContentLoaded", function () {
  const size = 500;
  const canvas = document.getElementById("canvas");
  canvas.width = size;
  canvas.style.width = size;
  canvas.height = size;
  canvas.style.height = size;
  const ctx = canvas.getContext("2d");

  let polygons = [
    // square
    [
      [0, 0],
      [0, size],
      [size, size],
      [size, 0],
      [0, 0],
    ],
    // L shape
    [
      [0, 0],
      [0, size],
      [size, size],
      [size, size / 2],
      [size / 2, size / 2],
      [size / 2, 0],
      [0, 0],
    ],
    // Triangle
    [
      [size / 2, 0],
      [0, size],
      [size, size],
      [size / 2, 0],
    ],
    // hexagon - good enough?
    [
      [size / 4, 0],
      [0, size / 2],
      [size / 4, size],
      [(3 * size) / 4, size],
      [size, size / 2],
      [(3 * size) / 4, 0],
      [size / 4, 0],
    ],
    // octagon
    [
      [(29 * size) / 100, 0],
      [0, (29 * size) / 100],
      [0, (71 * size) / 100],
      [(29 * size) / 100, size],
      [(71 * size) / 100, size],
      [size, (71 * size) / 100],
      [size, (29 * size) / 100],
      [(71 * size) / 100, 0],
      [(29 * size) / 100, 0],
    ],
    // Asteroid
    [
      [30, 30],
      [0, 180],
      [50, 410],
      [100, 490],
      [180, 390],
      [240, 490],
      [490, 450],
      [460, 420],
      [490, 320],
      [390, 120],
      [450, 20],
      [300, 0],
      [30, 30],
    ],
    // Star
    [
      [size / 2, 0],
      [(2 * size) / 5, size / 3],
      [0, size / 3],
      [size / 3, size / 2],
      [size / 5, size],
      [size / 2, (2 * size) / 3],
      [(4 * size) / 5, size],
      [(2 * size) / 3, size / 2],
      [size, size / 3],
      [(3 * size) / 5, size / 3],
      [size / 2, 0],
    ],
  ];
  const geoPolygons = [
    // geoPolygon1
    [
      {
        lat: -25.81972648546842,
        lng: -42.07594998386895,
      },
      {
        lat: -25.8205670552359,
        lng: -42.077108698163386,
      },
      {
        lat: -25.82187240035343,
        lng: -42.07568176296746,
      },
      {
        lat: -25.820853836675283,
        lng: -42.0762825777868,
      },
      {
        lat: -25.82164495475597,
        lng: -42.075349169049616,
      },
      {
        lat: -25.820804391642618,
        lng: -42.07446940449273,
      },
      {
        lat: -25.81972648546842,
        lng: -42.07594998386895,
      },
    ],
  ];
  polygons = polygons.concat(
    geoPolygons.map((geoP) => ocdots.buildPolygon(geoP, size, 0).polygon)
  );
  const DEFAULTN = 21;

  let N,
    mass,
    randomMasses,
    charge,
    randomCharges,
    baseForce,
    drag,
    viscosity,
    attenuation,
    maxMomentum,
    parallelForces,
    wallForces,
    points,
    momentum,
    polygon;

  function draw() {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    [points, momentum] = ocdots.movePoints({
      points,
      momentum,
      polygon,
      mass,
      charge,
      baseForce,
      drag,
      viscosity,
      maxMomentum,
      parallelForces,
      wallForces,
    });
    window.requestAnimationFrame(draw);
  }

  window.setValue = (variable, value) => {
    switch (variable) {
      case "baseForce":
        baseForce = value;
        break;
      case "drag":
        drag = value;
        break;
      case "viscosity":
        viscosity = value;
        break;
      case "maxMomentum":
        maxMomentum = value;
        break;
      case "parallelForces":
        parallelForces = value;
        break;
      case "wallForces":
        wallForces = value;
        break;
    }
    updateRange();
  };
  window.shakeem = () => {
    const shakeMax = 40;
    momentum = momentum.map((m) => [
      m[0] + shakeMax * Math.random() - shakeMax / 2,
      m[1] + shakeMax * Math.random() - shakeMax / 2,
    ]);
    for (let i = 0; i < 10; i++) {
      let m;
      [points, m] = ocdots.movePoints({
        points,
        momentum,
        polygon,
        baseForce: 0,
        drag: 0,
        maxMomentum: 10,
        parallelForces,
        wallForces,
      });
    }
  };
  window.resetPoints = () => {
    points = ocdots.randomInPolygon(N, polygon);
    momentum = points.map(() => [0, 0]);
    window.setMasses(randomMasses)
    window.setCharges(randomCharges)
  };

  window.setMasses = (random) => {
    if (random)
      mass = points.map(() => 0.01 + 3 * Math.random());
    else
      mass = ocdots.DEFAULTMASS;
  }

  window.setCharges = (random) => {
    if (random)
      charge = points.map(() => 3 * Math.random() - 1.5);
    else
      charge = ocdots.DEFAULTMASS;
  }

  window.resetControls = () => {
    N = DEFAULTN;
    mass = ocdots.DEFAULTMASS;
    charge = ocdots.DEFAULTCHARGE;
    baseForce = ocdots.BASEFORCE;
    drag = ocdots.DRAG;
    viscosity = ocdots.VISCOSITY;
    attenuation = ocdots.ATTENUATION;
    maxMomentum = ocdots.MAXMOMENTUM;
    parallelForces = ocdots.PARALLELFORCES;
    wallForces = ocdots.WALLFORCES;
    randomMasses = false;
    randomCharges = false;
    polygon = polygons[Math.floor(Math.random() * polygons.length)];
    window.resetPoints();
    document.getElementById("baseForce").value = baseForce;
    document.getElementById("drag").value = drag;
    document.getElementById("viscosity").value = viscosity;
    document.getElementById("maxMomentum").value = maxMomentum;
    document.getElementById("N").value = N;
    document.getElementById("parallelForces").checked = parallelForces;
    document.getElementById("randomMasses").checked = randomMasses;
    document.getElementById("randomCharges").checked = randomCharges;
    document.getElementById("wallForces").value = wallForces;
    updateRange();
  };
  function updateRange() {
    document.getElementById("baseForceValue").innerHTML = baseForce;
    document.getElementById("dragValue").innerHTML = drag;
    document.getElementById("viscosityValue").innerHTML = viscosity;
    document.getElementById("maxMomentumValue").innerHTML = maxMomentum;
    document.getElementById("wallForcesValue").innerHTML = wallForces;
  }

  document.getElementById("N").addEventListener("change", (event) => {
    N = event.srcElement.value;
    window.resetPoints();
  });

  document
    .getElementById("randomMasses")
    .addEventListener("change", (event) => {
      randomMasses = event.target.checked;
      window.setMasses(randomMasses);
    });

  document
    .getElementById("randomCharges")
    .addEventListener("change", (event) => {
      randomCharges = event.target.checked;
      window.setCharges(randomCharges);
    });

  window.resetControls();
  draw();
});
