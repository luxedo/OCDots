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
export function drawPoints(
  ctx,
  points,
  mass,
  charge,
  baseRadius = 3,
  palette = ["#000000"]
) {
  const _mass = Array.isArray(mass)
    ? [...mass]
    : new Array(points.length).fill(1);

  const _charge = Array.isArray(charge)
    ? [...charge]
    : new Array(points.length).fill(1);
  const maxCharge = _charge.reduce((acc, cur) => cur > acc ? cur : acc);
  const minCharge = _charge.reduce((acc, cur) => cur < acc ? cur : acc);
  const deltaCharge = (maxCharge - minCharge)/palette.length;
  let _palette = [...palette];
  if (minCharge == maxCharge)
    _palette = [palette[Math.floor(palette.length/2)]]

  points.forEach((pt, i) => {
    let color = palette[palette.length-1];
    for (let j=0; j<palette.length; j++) {
      if (_charge[i] <= minCharge + deltaCharge * (j+1)) {
        color = _palette[j];
        break;
      }
    }
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pt[0], pt[1], 3 + _mass[i] * baseRadius, 0, 2 * Math.PI, 0);
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
  mass,
  charge,
  baseRadius = 3,
  palette = ["#881111", "#000000", "#111188"],
  backgroundColor = "#FFFFFF"
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
  drawPoints(ctx, points, mass, charge, baseRadius, palette);
  drawPolygon(ctx, polygon, "#000000");
}
