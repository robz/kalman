/*
 * This is a simple example of how to use the kalman filter, where the state
 * is directly being measured, is a single value, and is constant.
 *
 * In this senario, a running average would be just as effective as  Kalman
 * filter.
 *
 * @flow
 */

const KalmanFilter = require('./KalmanFilter');
const {index, matrix, subset} = require('mathjs');
const {makeCanvasFitWindow, normal} = require('./utils');

const MEAN = 1.3;
const STEPS = 50;
const VARIANCE = .1;

// hack to get value out 1x1 mathjs matrix
function getScalar(m) {
  try {
    return subset(m, index(0,0));
  } catch(e) {
    // Apparently mutliplying two matricies together can yield a scalar
    return m;
  }
}

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

  const Chart = require('chart.js');

  makeCanvasFitWindow('canvas');
  let canvas: any = document.getElementById('canvas');
  let ctx = canvas.getContext('2d');

  let data = {
    labels: stateData.map((e,i) => i),
    datasets: [
      {
        label: 'Truth',
        fillColor: 'rgba(205,187,151,0.2)',
        strokeColor: 'rgba(205,187,151,1)',
        pointColor: 'rgba(205,187,151,1)',
        pointStrokeColor: '#fff',
        pointHighlightFill: '#fff',
        pointHighlightStroke: 'rgba(205,187,151,1)',
        data: trueData,
      },
      {
        label: 'Measurements',
        fillColor: 'rgba(220,220,220,0.2)',
        strokeColor: 'rgba(220,220,220,1)',
        pointColor: 'rgba(220,220,220,1)',
        pointStrokeColor: '#fff',
        pointHighlightFill: '#fff',
        pointHighlightStroke: 'rgba(220,220,220,1)',
        data: measurementData,
      },
      {
        label: 'States',
        fillColor: 'rgba(151,187,205,0.2)',
        strokeColor: 'rgba(151,187,205,1)',
        pointColor: 'rgba(151,187,205,1)',
        pointStrokeColor: '#fff',
        pointHighlightFill: '#fff',
        pointHighlightStroke: 'rgba(151,187,205,1)',
        data: stateData,
      },
    ],
  };

  let myLineChart = new Chart(ctx).Line(data, {bezierCurve: false});
}

module.exports = example;
