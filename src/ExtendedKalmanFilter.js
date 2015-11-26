/* @flow */

const {eye, clone, format, add, subtract,  inv, transpose, matrix} = require('mathjs');
const {multiply} = require('./utils');

type Matrix = Object;
type Params = {
  MeasurementCovariance: Matrix;
  ObservationFunction: (x: Matrix) => Matrix;
  ObservationJacobian: Matrix;
  ProcessCovariance: Matrix;
  StateTransitionFunction: (x: Matrix, u: Matrix) => Matrix;
  StateTransitionJacobian: Matrix;
};

class ExtendedKalmanFilter {
  State: Matrix;
  StateCovariance: Matrix;
  params: Params;
  NUM_DIMENSIONS: number;

  constructor(
    InitialCovariance: Matrix,
    InitialState: Matrix,
    params: Params;
  ) {
    this.State = InitialState;
    this.StateCovariance = InitialCovariance;
    this.params = params;
    this.NUM_DIMENSIONS = InitialState.size()[0];
  }

  step(
    MeasurementInput: Matrix,
    ControlInput: Matrix,
  ): {State: Matrix; StateCovariance: Matrix} {
    let {
      State: x,
      StateCovariance: P,
      params: {
        MeasurementCovariance: R,
        ObservationFunction: h,
        ObservationJacobian: H,
        ProcessCovariance: Q,
        StateTransitionFunction: f
        StateTransitionJacobian: F,
      },
      NUM_DIMENSIONS: N,
    } = this;

    let u = ControlInput;
    let z = MeasurementInput;

    // predict
    let xPriori = f(x, u);
    let PPriori = add(Q, multiply(F, P, transpose(F)));

    // measurement and innovation
    let y = subtract(z, h(xPriori));
    let S = add(R, multiply(H, PPriori, transpose(H)));

    // kalman gain
    let K = multiply(PPriori, transpose(H), inv(S));

    // update
    let xPosteriori = add(xPriori, multiply(K, y));
    let PPosteriori = multiply(subtract(eye(N), multiply(K, H)), PPriori);

    this.State = xPosteriori;
    this.StateCovariance = PPosteriori;

    return {State: this.State, StateCovariance: this.StateCovariance};
  }
}

module.exports = ExtendedKalmanFilter;
