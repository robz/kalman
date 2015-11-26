/* @flow */

//let example = require('./example1.js');
let example = require('./example2.js');
example();

function benchmark() {
  let steps = 1000;
  let errorSum = 0;
  for (let i = 0; i < steps; i++) {
    errorSum += example();
  }

  console.log('average error:', errorSum / steps);
}

//benchmark();
