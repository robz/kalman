/*
 * This is the same sinusoid simulation as example 3, with the same measurement
 * input, but instead of estimating position and velocity, we estimate the
 * characteristics of the sinusoid (frequency, amplitude, and phase).
 *
 * However, we can't use a normal kalman filter here, because the transform
 * from the state to the measurement is nonlinear:
 *
 *   measurement = h(state)
 *               = amp * sin( freq * t + phase )
 *
 * One way to handle this nonlinear function is to use an extended kalman
 * filter.
 *
 * @flow
 */

const ExtendedKalmanFilter = require('./ExtendedKalmanFilter');
const {PI, matrix, cos, sin} = require('mathjs');
const {normal} = require('./utils');

const DT = 1;
const STEPS = 100;

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

  let trueData = [0]; // ignore this first value
  let measurementData = [0]; // ignore this first measurement
  let x = kalmanFilter.State._data;
  let stateData = [x[0][0] *  sin(x[2][0] + x[1][0] * x[3][0])];

  for (let i = 0; i < STEPS; i++) {
    // simulate
    let t = DT * i;
    let position = MAGNITUDE * sin(t * 2 * PI / PERIOD + 1);

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
    measurementData.push(MeasurementInput._data[0][0]);
    x = State._data;
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
