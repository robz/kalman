/*
 * This is the same as example 2, but instead of the process being a constant,
 * linear function, it's a sinusoid.
 *
 */

const KalmanFilter = require('./KalmanFilter');
const {PI, abs, index, matrix, multiply, pow, sin, subset, ones} = require('mathjs');
const {getScalar, normal} = require('./utils');

const DT = 1;
const STEPS = 1000;

const MEASUREMENT_VARIANCE = 10;

// Here, we need to account for the fact that the model does not represent
// the process very well.
const PROCESS_VARIANCE = .01;

const MAGNITUDE = 10;
const PERIOD = DT * STEPS / 6;

function example(): number {
  /*
   * Kalman filter steps
   */
  let kalmanFilter = new KalmanFilter({
    ControlModel: matrix([
      [0],
      [0],
    ]),
    InitialCovariance: matrix([
      [99, 0],
      [0, 99],
    ]),
    InitialState: matrix([
      [0],
      [0],
    ]),
    MeasurementCovariance: matrix([
      [MEASUREMENT_VARIANCE],
    ]),
    ObservationModel: matrix([
      [1, 0],
    ]),
    ProcessCovariance: matrix([
      [PROCESS_VARIANCE, 0],
      [0, PROCESS_VARIANCE],
    ]),
    StateTransitionModel: matrix([
      [1, DT],
      [0, 1],
    ]),
  });

  let measurementData = [null]; // ignore this first measurement
  let stateData = [[
    getScalar(kalmanFilter.State, 0, 0),
    getScalar(kalmanFilter.State, 1, 0),
  ]];
  let covarianceData = [getScalar(kalmanFilter.StateCovariance, 0, 0)];
  let trueData = [null];
  let position = 0;
  let stepData = [0];

  for (let i = 0; i < STEPS; i++) {
    position = MAGNITUDE * sin(DT * i * 2 * PI / PERIOD);

    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(position, MEASUREMENT_VARIANCE)]]);
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    trueData.push(position);
    measurementData.push(getScalar(MeasurementInput, 0, 0));
    stateData.push([
      getScalar(State, 0, 0),
      getScalar(State, 1, 0),
    ]);
    covarianceData.push(getScalar(StateCovariance, 0, 0));
  }

  let totalError = 0;
  measurementData.forEach((_, i) => {
    if (i !== 0) {
      totalError += abs(stateData[i][0] - trueData[i]);
    }
    if (global.window) {
      //console.log(trueData[i], measurementData[i], stateData[i], covarianceData[i]);
    }
  });

  /*
   * Draw the data
   */
  if (!global.window) return totalError;

  const plotly = require('plotly.js');

  const xs = stateData.map((e,i) => i);
  stateData = stateData.map(e => e[0]);

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
