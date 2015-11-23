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
const {abs, index, matrix, multiply, pow, subset, transpose} = require('mathjs');
const {makeCanvasFitWindow, normal} = require('./utils');

const DT = 1;
const START = 1.3;
const STEPS = 50;
const MEASUREMENT_VARIANCE = .2;
const VELOCITY = .1;
const PROCESS_VARIANCE = .0001;

const ACCELERATION = matrix([
  [.5 * DT * DT],
  [DT],
]);

// hack to get value out 1x1 mathjs matrix
function getScalar(m: Object, row: number, col: number) {
  try {
    return subset(m, index(row, col));
  } catch(e) {
    // Apparently mutliplying two matricies together can yield a scalar
    return m;
  }
}

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
      [999, 0],
      [0, 999],
    ]),
    InitialState: matrix([
      [0],
      [0],
    ]),
    MeasurementCovariance: matrix([
      [.2],
    ]),
    ObservationModel: matrix([
      [1, 0],
    ]),
    ProcessCovariance: multiply(
      pow(PROCESS_VARIANCE, 2),
      multiply(ACCELERATION, transpose(ACCELERATION))
    ),
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
  let position = START;

  for (let i = 0; i < STEPS; i++) {
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

    position += VELOCITY * DT;
  }

  let totalError  = 0;
  measurementData.forEach((_, i) => {
    //console.log(measurementData[i], stateData[i], covarianceData[i]);
    if (i === 0) return;
    totalError += abs(stateData[i][0] - trueData[i]);
  });
  //return totalError;


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
        data: stateData.map(state => state[0]),
      },
    ],
  };

  let myLineChart = new Chart(ctx).Line(data, {bezierCurve: false});
}

module.exports = example;
