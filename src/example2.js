/*
 * This is another simple example of how to use the kalman filter, where the
 * state include a position and velocity, only the position is being measured,
 * and the velocity is assumed to be constant.
 *
 * Since the velocity is constant, it's probably just as efficient to use a
 * running average or linear regression to determine the state.
 *
 * @flow
 */

const KalmanFilter = require('./KalmanFilter');
const {index, matrix, multiply, pow, subset, ones} = require('mathjs');
const {makeCanvasFitWindow, normal} = require('./utils');

const DT = 1;
const START = 1.3;
const STEPS = 50;
const MEASUREMENT_VARIANCE = .4;
const VELOCITY = .1;

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
      [MEASUREMENT_VARIANCE],
    ]),
    ObservationModel: matrix([
      [1, 0],
    ]),
    ProcessCovariance: multiply(
      PROCESS_VARIANCE,
      ones(2, 2)
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
        data: stateData.map(state => state[0]),
      },
    ],
  };

  let myLineChart = new Chart(ctx).Line(data, {bezierCurve: false});
}

module.exports = example;
