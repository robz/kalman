/*
 * This is another simple example of how to use the kalman filter, where the
 * state include a position and velocity, only the position is being measured,
 * and the velocity is assumed to be constant.
 *
 * Since the velocity is constant, it's probably just as efficient to use a
 * running average or linear regression to determine the state.
 *
 * @flow
 *
 */

const KalmanFilter = require('./KalmanFilter');
const {abs, matrix, multiply, ones} = require('mathjs');
const {normal} = require('./utils');

const DT = 1;
const STEPS = 50;

const MEASUREMENT_VARIANCE = .4;

/*
 * Some examples I've found use the next derivative of a state to estimate the
 * process variance, to simulate unknown forces. In this case, that would be
 * acceleration, since we're already estimating position and velocity.
 *
 * Seems to me that if the force is truly unknown and random, it doesn't much
 * matter what the process variance is, so long it increases as DT increases.
 * (Since the longer you go without measuring, the more uncertain you should
 * be with your state estimation)
 *
 * In our simulation here, the state model precisely reflects the process, so
 * this value can just be zero.
 */
const PROCESS_VARIANCE = 0;

const START = 1.3;
const VELOCITY = .1;

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
      [PROCESS_VARIANCE, PROCESS_VARIANCE],
      [PROCESS_VARIANCE, PROCESS_VARIANCE],
    ]),
    StateTransitionModel: matrix([
      [1, DT],
      [0, 1],
    ]),
  });

  let trueData = [0]; // ignore this first value
  let measurementData = [0]; // ignore this first measurement
  let x = kalmanFilter.State._data;
  let stateData = [[x[0][0], x[1][0]]];
  let covarianceData = [kalmanFilter.StateCovariance._data[0][0]];

  let position = START;

  for (let i = 0; i < STEPS; i++) {
    // simulate
    position = START + i * DT * VELOCITY;

    // measure
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(position, MEASUREMENT_VARIANCE)]]);

    // step
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    // record
    trueData.push(position);
    measurementData.push(MeasurementInput._data[0][0]);
    let x = kalmanFilter.State._data;
    stateData.push([x[0][0], x[1][0]]);
    covarianceData.push(StateCovariance._data[0][0]);
  }

  if (!global.window) {
    let totalError = 0;
    trueData.forEach((_, i) => {
      if (i !== 0) {
        totalError += abs(stateData[i][0] - trueData[i]);
      }
    });
    return totalError;
  } else {
    //trueData.forEach((_, i) => {
    //  console.log(trueData[i], measurementData[i], stateData[i], covarianceData[i]);
    //});
  }

  /*
   * Draw the data
   */
  const plotly = require('plotly.js');

  const xs = stateData.map((e,i) => i);
  stateData = stateData.map(e => e[0]);

  plotly.newPlot(
    'myDiv',
    [
       {x: xs, y: trueData, mode: 'lines', type: 'scatter'},
       {x: xs, y: measurementData, mode: 'lines', type: 'scatter'},
       {x: xs, y: stateData, mode: 'lines', type: 'scatter'},
    ]
  );

  return 0;
}

module.exports = example;
