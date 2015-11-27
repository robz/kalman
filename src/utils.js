/* @flow */

var {subset, multiply: multiplyTwo, index} = require('mathjs');
var {PI, sqrt, log, random, cos} = Math;

export type Matrix = Object;

// matrix multiply with any number of matricies
function multiply(...args: Array<Matrix>): Matrix {
  if (args.length < 2) {
    throw new Error('multiply must take at least 2 arguments');
  }

  let result = multiplyTwo(args[0], args[1]);
  for (let i = 2; i < args.length; i++) {
    // matrix multiplication is associative, so A*B*C = (A*B)*C
    result = multiplyTwo(result, args[i]);
  }

  return result;
}

// hack to get value out 1x1 mathjs matrix
function getScalar(m: Object, row?: number = 0, col?: number = 0) {
  try {
    return subset(m, index(row, col));
  } catch(e) {
    // Apparently mutliplying two matricies together can yield a scalar
    return m;
  }
}

function normal(
  mean?: number = 0,
  variance?: number = 1
): number {
  // https://en.wikipedia.org/wiki/Box-Muller_transform
  var unitNormal = sqrt(-2 * log(random())) * cos(2 * PI * random());
  return mean + sqrt(variance) * unitNormal;
}

function makeCanvasFitWindow(canvasID: string): void {
  var canvas: any = document.getElementById(canvasID);
  var context: any = canvas.getContext('2d');

  var dpr = window.devicePixelRatio || 1;
  var bsr =
    context.webkitBackingStorePixelRatio ||
    context.mozBackingStorePixelRatio ||
    context.msBackingStorePixelRatio ||
    context.oBackingStorePixelRatio ||
    context.backingStorePixelRatio ||
    1;
  var PIXEL_RATIO = dpr / bsr;

  var width = window.innerWidth;
  var height = window.innerHeight;
  canvas.width = width * PIXEL_RATIO;
  canvas.height = height * PIXEL_RATIO;
  canvas.style.width = width + "px";
  canvas.style.height = height + "px";
  context.setTransform(PIXEL_RATIO, 0, 0, PIXEL_RATIO, 0, 0);
}

module.exports = {getScalar, multiply, normal, makeCanvasFitWindow};
