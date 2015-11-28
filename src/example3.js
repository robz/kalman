/*
 * This is the same as example 2, but instead of the process being a constant,
 * linear function, it's a sinusoid.
 *
 * @flow
 *
 */

const KalmanFilter = require('./KalmanFilter');
const {PI, abs, matrix, sin} = require('mathjs');
const {normal} = require('./utils');

const DT = 1;
const STEPS = 1000;

const MEASUREMENT_VARIANCE = 10;

/*
 * Here, we need to account for the fact that the model does not represent
 * the process very well. I believe it should also be a function of DT for
 * two intuitive reasons:
 *
 * 1) the longer you go without a measurement, the more uncertainer you are
 * 2) if you do two predictions in a time step, and then one measurement, you
 *    should be just as certain as if you made one prediction, and then one
 *    measurement.
 */
const PROCESS_VARIANCE = .01 * DT;

const MAGNITUDE = 10;
const PERIOD = DT * STEPS / 4;

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

  let trueData = [0]; // ignore this first value
  let measurementData = [0]; // ignore this first measurement
  let x = kalmanFilter.State._data;
  let stateData = [[x[0][0], x[1][0]]];
  let covarianceData = [kalmanFilter.StateCovariance._data[0][0]];

  let position = 0;

  for (let i = 0; i < STEPS; i++) {
    // simulate
    position = MAGNITUDE * sin(DT * i * 2 * PI / PERIOD);

    // measure
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(position, MEASUREMENT_VARIANCE)]]);

    // step
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    // record
    trueData.push(position);
    measurementData.push(MeasurementInput._data[0][0]);
    x = State._data;
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
      {x: xs, y: trueData, mode: 'lines', type: 'scatter', name: 'true'},
      {x: xs, y: measurementData, mode: 'lines', type: 'scatter', name: 'measurement'},
      {x: xs, y: stateData, mode: 'lines', type: 'scatter', name: 'estimate'},
    ]
  );

  return 0;
}

module.exports = example;
