import * as ocdots from "./ocdots.js";
import { drawPolygonAndPoints } from "./draw.js";

let randomMasses,
  randomCharges,
  positiveCharges,
  baseForce,
  drag,
  viscosity,
  maxMomentum,
  parallelForces,
  wallForces;

document.addEventListener("DOMContentLoaded", function () {
  loadMainCanvas();
  loadPointDemo();
  loadWireDemo();
  loadDragViscDemo();
  loadMassChargeDemo();
  draw();
});

function loadMainCanvas() {
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

  let N, mass, charge, points, momentum, polygon, shakeTicks;

  window.drawMain = () => {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    let _momentum;
    [points, _momentum] = ocdots.movePoints({
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
    if (shakeTicks == 0) {
      momentum = _momentum;
    } else {
      shakeTicks--;
    }
  };

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
    shakeTicks = 10;
    const shakeMax = 10;
    momentum = momentum.map((m) => [
      m[0] + shakeMax * Math.random() - shakeMax / 2,
      m[1] + shakeMax * Math.random() - shakeMax / 2,
    ]);
  };
  window.resetPoints = () => {
    points = ocdots.randomInPolygon(N, polygon);
    momentum = points.map(() => [0, 0]);
    window.setMasses(randomMasses);
    window.setCharges(randomCharges, positiveCharges);
  };

  window.setMasses = (random) => {
    if (random) mass = points.map(() => 0.1 + 3 * Math.random());
    else mass = ocdots.DEFAULTMASS;
  };

  window.setCharges = (random, positive) => {
    if (random && !positive) charge = points.map(() => 3 * Math.random() - 1.5);
    else if (random && positive) charge = points.map(() => 3 * Math.random());
    else if (!random && !positive)
      charge = points.map(() => (Math.random() > 0.5 ? 1 : -1));
    else charge = ocdots.DEFAULTMASS;
  };

  window.resetControls = () => {
    N = DEFAULTN;
    mass = ocdots.DEFAULTMASS;
    charge = ocdots.DEFAULTCHARGE;
    baseForce = ocdots.BASEFORCE;
    drag = ocdots.DRAG;
    viscosity = ocdots.VISCOSITY;
    maxMomentum = ocdots.MAXMOMENTUM;
    parallelForces = ocdots.PARALLELFORCES;
    wallForces = ocdots.WALLFORCES;
    randomMasses = false;
    randomCharges = false;
    positiveCharges = true;
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
    document.getElementById("positiveCharges").checked = positiveCharges;
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
      window.setCharges(randomCharges, positiveCharges);
    });

  document
    .getElementById("positiveCharges")
    .addEventListener("change", (event) => {
      positiveCharges = event.target.checked;
      window.setCharges(randomCharges, positiveCharges);
    });

  window.resetControls();
}

function loadPointDemo() {
  const width = 300;
  const height = 100;
  const pad = 100;
  const canvas = document.getElementById("point-demo");
  canvas.width = width;
  canvas.style.width = width;
  canvas.height = height;
  canvas.style.height = height;
  const ctx = canvas.getContext("2d");

  let points, momentum, mass, charge;
  function resetPointDemo() {
    points = [
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
    ];
    momentum = points.map(() => [
      2 * maxMomentum * Math.random() - maxMomentum,
      2 * maxMomentum * Math.random() - maxMomentum,
    ]);

    mass = randomMasses
      ? points.map(() => 0.1 + 3 * Math.random())
      : ocdots.DEFAULTMASS;

    if (randomCharges && !positiveCharges)
      charge = points.map(() => 3 * Math.random() - 1.5);
    else if (randomCharges && positiveCharges)
      charge = points.map(() => 3 * Math.random());
    else if (!randomCharges && !positiveCharges)
      charge = points.map(() => (Math.random() > 0.5 ? 1 : -1));
    else charge = ocdots.DEFAULTMASS;
  }
  let polygon = [
    [-pad, -pad],
    [-pad, height + pad],
    [width + pad, height + pad],
    [width + pad, -pad],
  ];
  resetPointDemo();

  window.drawPointDemo = () => {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    let _momentum;
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
      wallForces: 6,
    });
  };
  setInterval(resetPointDemo, 3000 + 1000 * Math.random());
}

function loadWireDemo() {
  const width = 300;
  const height = 100;
  const pad = 100;
  const canvas = document.getElementById("wire-demo");
  canvas.width = width;
  canvas.style.width = width;
  canvas.height = height;
  canvas.style.height = height;
  const ctx = canvas.getContext("2d");

  let points, momentum, mass, charge;
  function resetWireDemo() {
    points = [
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
    ];
    momentum = points.map(() => [
      2 * maxMomentum * Math.random() - maxMomentum,
      2 * maxMomentum * Math.random() - maxMomentum,
    ]);

    mass = randomMasses
      ? points.map(() => 0.1 + 3 * Math.random())
      : ocdots.DEFAULTMASS;

    if (randomCharges && !positiveCharges)
      charge = points.map(() => 3 * Math.random() - 1.5);
    else if (randomCharges && positiveCharges)
      charge = points.map(() => 3 * Math.random());
    else if (!randomCharges && !positiveCharges)
      charge = points.map(() => (Math.random() > 0.5 ? 1 : -1));
    else charge = ocdots.DEFAULTMASS;
  }
  let polygon = [
    [0, 0],
    [0, height],
    [width, height],
    [width, 0],
  ];
  resetWireDemo();

  window.drawWireDemo = () => {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    let _momentum;
    [points, momentum] = ocdots.movePoints({
      points,
      momentum,
      polygon,
      mass,
      charge,
      baseForce,
      drag: 0.1 * drag,
      viscosity,
      maxMomentum,
      parallelForces,
      wallForces: 0.1,
    });
  };
  setInterval(resetWireDemo, 3000 + 1000 * Math.random());
}

function loadDragViscDemo() {
  const width = 300;
  const height = 100;
  const pad = 100;
  const canvas = document.getElementById("drag-visc-demo");
  canvas.width = width;
  canvas.style.width = width;
  canvas.height = height;
  canvas.style.height = height;
  const ctx = canvas.getContext("2d");

  let points, momentum, mass, charge;
  function resetDragViscDemo() {
    points = [
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
    ];
    momentum = points.map(() => [
      2 * maxMomentum * Math.random() - maxMomentum,
      2 * maxMomentum * Math.random() - maxMomentum,
    ]);

    mass = randomMasses
      ? points.map(() => 0.1 + 3 * Math.random())
      : ocdots.DEFAULTMASS;

    if (randomCharges && !positiveCharges)
      charge = points.map(() => 3 * Math.random() - 1.5);
    else if (randomCharges && positiveCharges)
      charge = points.map(() => 3 * Math.random());
    else if (!randomCharges && !positiveCharges)
      charge = points.map(() => (Math.random() > 0.5 ? 1 : -1));
    else charge = ocdots.DEFAULTMASS;
  }
  let polygon = [
    [0, 0],
    [0, height],
    [width, height],
    [width, 0],
  ];
  resetDragViscDemo();

  window.drawDragViscDemo = () => {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    let _momentum;
    [points, momentum] = ocdots.movePoints({
      points,
      momentum,
      polygon,
      mass,
      charge,
      baseForce,
      drag: 0.5,
      viscosity: 0.9,
      maxMomentum,
      parallelForces,
      wallForces: 0.1,
    });
  };
  setInterval(resetDragViscDemo, 3000 + 1000 * Math.random());
}

function loadMassChargeDemo() {
  const width = 300;
  const height = 100;
  const pad = 100;
  const canvas = document.getElementById("mass-charge-demo");
  canvas.width = width;
  canvas.style.width = width;
  canvas.height = height;
  canvas.style.height = height;
  const ctx = canvas.getContext("2d");

  let points, momentum, mass, charge;
  function resetMassChargeDemo() {
    points = [
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
      [width / 2 + 10 * Math.random() - 5, height / 2 + 10 * Math.random() - 5],
    ];
    momentum = points.map(() => [
      2 * maxMomentum * Math.random() - maxMomentum,
      2 * maxMomentum * Math.random() - maxMomentum,
    ]);

    mass = points.map(() => 0.1 + 3 * Math.random())
    charge = points.map(() => (Math.random() > 0.5 ? 1 : -1));
  }
  let polygon = [
    [0, 0],
    [0, height],
    [width, height],
    [width, 0],
  ];
  resetMassChargeDemo();

  window.drawMassChargeDemo = () => {
    drawPolygonAndPoints(canvas, points, polygon, mass, charge);
    let _momentum;
    [points, momentum] = ocdots.movePoints({
      points,
      momentum,
      polygon,
      mass,
      charge,
      baseForce: 2,
      drag,
      viscosity,
      maxMomentum,
      parallelForces,
      wallForces: 0.1,
    });
  };
  setInterval(resetMassChargeDemo, 3000 + 1000 * Math.random());
}

function draw() {
  drawMain();
  drawPointDemo();
  drawWireDemo();
  drawDragViscDemo();
  drawMassChargeDemo();
  window.requestAnimationFrame(draw);
}
