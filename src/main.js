/* @flow */

let example = require('./example3.js');
example();

function benchmark() {
  let steps = 100;
  let errorSum = 0;
  for (let i = 0; i < steps; i++) {
    errorSum += example();
  }

  console.log('average error:', errorSum / steps);
}

if (!global.window) {
  benchmark();
}
