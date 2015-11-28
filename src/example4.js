/*
 * This is the same as example 2, but instead of the process being a constant,
 * linear function, it's a sinusoid.
 *
 * @flow
 */

const ExtendedKalmanFilter = require('./ExtendedKalmanFilter');
const {PI, abs, index, matrix, multiply, pow, cos, sin, subset, ones} = require('mathjs');
const {getScalar, normal} = require('./utils');

const DT = 1;
const STEPS = 1000;

const MEASUREMENT_VARIANCE = 10;
const PROCESS_VARIANCE = 0;

const MAGNITUDE = 10;
const PERIOD = DT * STEPS / 3;

function example(): void {
  /*
   * Kalman filter steps
   */
  let kalmanFilter = new ExtendedKalmanFilter({
    InitialCovariance: matrix([
      [1e12, 0, 0, 0],
      [0, 1e12, 0, 0],
      [0, 0, 1e12, 0],
      [0, 0, 0, 0],
    ]),
    InitialState: matrix([
      [5],
      [1],
      [0],
      [0],
    ]),
    params: {
      MeasurementCovariance: matrix([
        [MEASUREMENT_VARIANCE, 0],
        [0, 1e-12],
      ]),
      observationFunct: ([[a],[w],[d],[t]]) => matrix([
        [a * sin(d + w * t)],
        [t],
      ]),
      observationJacobianFunct: ([[a],[w],[d],[t]]) => matrix([
        [sin(d + w * t), a * t * cos(d + w * t), a * cos(d + w * t), w * cos(d + w * t)],
        [0, 0, 0, 1],
      ]),
      ProcessCovariance: matrix([
        [PROCESS_VARIANCE, 0, 0, 0],
        [0, PROCESS_VARIANCE, 0, 0],
        [0, 0, PROCESS_VARIANCE, 0],
        [0, 0, 0, 0],
      ]),
      stateTransitionFunct: ([[a],[w],[d],[t]]) => matrix([
        [a],
        [w],
        [d],
        [t + DT],
      ]),
      stateTransitionJacobianFunct: ([[a],[w],[d],[t]]) => matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
      ]),
    }
  });

  let measurementData = [0];
  let stateData = [0]
  let covarianceData = [getScalar(kalmanFilter.StateCovariance, 0, 0)];
  let trueData = [0];

  for (let i = 0; i < STEPS; i++) {
    // simulate
    let t = DT * i;
    let position = MAGNITUDE * sin(t * 2 * PI / PERIOD);

    // measure
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([
      [normal(position, MEASUREMENT_VARIANCE)],
      [t],
    ]);

    // step
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    // record
    trueData.push(position);
    measurementData.push(getScalar(MeasurementInput, 0, 0));
    let x = State._data;
    stateData.push(x[0][0] *  sin(x[2][0] + x[1][0] * x[3][0]));
  }

  /*
   * Draw the data
   */
  const plotly = require('plotly.js');

  const xs = stateData.map((e,i) => i);

  plotly.newPlot(
    'myDiv',
    [
      {x: xs, y: trueData, mode: 'lines', type: 'scatter', name: 'true'},
      {x: xs, y: measurementData, mode: 'lines', type: 'scatter', name: 'measurement'},
      {x: xs, y: stateData, mode: 'lines', type: 'scatter', name: 'estimate'},
    ]
  );
}

module.exports = example;
