/* @flow */

var {PI, sqrt, log, random, cos} = Math;

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

module.exports = {normal, makeCanvasFitWindow};
