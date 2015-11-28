/* @flow */

let example = require('./example1.js');
example();

function benchmark() {
  let steps = 100;
  let errorSum = 0;
  for (let i = 0; i < steps; i++) {
    errorSum += example();
  }

  console.log(`average error over ${steps} steps:`, errorSum / steps);
}

if (!global.window) {
  benchmark();
}
