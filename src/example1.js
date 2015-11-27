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

const MEAN = 1.3;
const STEPS = 50;
const VARIANCE = .1;

// hack to get value out 1x1 mathjs matrix
function example() {
  /*
   * Kalman filter steps
   */
  let kalmanFilter = new KalmanFilter({
    ControlModel: matrix([[0]]),            // no control
    InitialCovariance: matrix([[1]]),       // guess
    InitialState: matrix([[3]]),            // guess
    MeasurementCovariance: matrix([[.2]]),  // >= VARIANCE
    ObservationModel: matrix([[1]]),        // state = measurement
    ProcessCovariance: matrix([[.00001]]),  // guess
    StateTransitionModel: matrix([[1]]),    // state doesn't change
  });

  let measurementData = [null]; // ignore this first measurement
  let stateData = [getScalar(kalmanFilter.State)];
  let covarianceData = [getScalar(kalmanFilter.StateCovariance)];
  let trueData = [null];

  for (let i = 0; i < STEPS; i++) {
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(MEAN, VARIANCE)]]);
    let {State, StateCovariance} =
      kalmanFilter.step(MeasurementInput, ControlInput);

    trueData.push(MEAN);
    measurementData.push(getScalar(MeasurementInput));
    stateData.push(getScalar(State));
    covarianceData.push(getScalar(StateCovariance));
  }

  measurementData.forEach((_, i) => {
    console.log(measurementData[i], stateData[i], covarianceData[i]);
  });

  /*
   * Draw the data
   */
  if (!global.window) return;

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
