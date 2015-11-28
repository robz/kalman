/*
 * This is a simple example of how to use the kalman filter, where the state
 * is directly being measured, is a single value, and is constant.
 *
 * In this senario, a running average would be just as effective as  Kalman
 * filter.
 *
 */

const KalmanFilter = require('./KalmanFilter');
const {matrix} = require('mathjs');
const {getScalar, normal} = require('./utils');

const STEPS = 50;
const MEAN = 1.3;
const MEASUREMENT_VARIANCE = .1;

// hack to get value out 1x1 mathjs matrix
function example() {
  /*
   * Kalman filter steps
   */
  let kalmanFilter = new KalmanFilter({
    ControlModel: matrix([[0]]),            // no control
    InitialCovariance: matrix([[1]]),       // guess
    InitialState: matrix([[3]]),            // guess
    MeasurementCovariance: matrix([[MEASUREMENT_VARIANCE]]),
    ObservationModel: matrix([[1]]),        // state = measurement
    ProcessCovariance: matrix([[0]]),       // model is the simulation
    StateTransitionModel: matrix([[1]]),    // state doesn't change
  });

  let trueData = [0]; // ignore this first value
  let measurementData = [0]; // ignore this first measurement
  let stateData = [getScalar(kalmanFilter.State)];
  let covarianceData = [getScalar(kalmanFilter.StateCovariance)];

  for (let i = 0; i < STEPS; i++) {
    // measure
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(MEAN, MEASUREMENT_VARIANCE)]]);

    // step
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    // record
    trueData.push(MEAN);
    measurementData.push(getScalar(MeasurementInput));
    stateData.push(getScalar(State));
    covarianceData.push(getScalar(StateCovariance));
  }

  measurementData.forEach((_, i) => {
    console.log(measurementData[i], stateData[i], covarianceData[i]);
  });

  if (!global.window) return;

  /*
   * Draw the data
   */
  const plotly = require('plotly.js');

  var xs = stateData.map((e,i) => i);

  plotly.newPlot(
    'myDiv',
    [
       {x: xs, y: trueData, mode: 'lines', type: 'scatter'},
       {x: xs, y: measurementData, mode: 'lines', type: 'scatter'},
       {x: xs, y: stateData, mode: 'lines', type: 'scatter'},
    ]
  );
}

module.exports = example;
