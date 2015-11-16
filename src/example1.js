/*
 * This is a simple example of how to use the kalman filter, where the state
 * is directly being measured, is a single value, and is constant.
 *
 * In this senario, using a running average would be just as effective as
 * using a Kalman filter.
 *
 * @flow
 */

const KalmanFilter = require('./KalmanFilter');
const Chart = require('chart.js');
const {format, matrix, subset, index} = require('mathjs');
const {normal, makeCanvasFitWindow} = require('./utils');

let STEPS = 60;
let MEAN = 1.3;
let VARIANCE = .1;

// hack to get value out of mathjs matrix
function getValue(m) {
  try {
    return subset(m, index(0,0));
  } catch(e) {
    // Apparently mutliplying two matries together can yield a scalar
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

  let measurementData = [0]; // ignore this first measurement
  let stateData = [getValue(kalmanFilter.PreviousState)];
  let covarianceData = [getValue(kalmanFilter.PreviousCovariance)];

  for (let i = 0; i < STEPS; i++) {
    let ControlInput = matrix([[0]]); // no control
    let MeasurementInput = matrix([[normal(MEAN, VARIANCE)]]);
    kalmanFilter.step(MeasurementInput, ControlInput);

    measurementData.push(getValue(MeasurementInput));
    stateData.push(getValue(kalmanFilter.PreviousState));
    covarianceData.push(getValue(kalmanFilter.PreviousCovariance));
  }


  /*
   * Draw the data
   */

  makeCanvasFitWindow('canvas');
  let canvas = document.getElementById('canvas');
  let ctx = canvas.getContext('2d');

  let data = {
    labels: stateData.map((e,i) => i),
    datasets: [
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
