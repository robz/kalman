/* @flow */

import type {Matrix} from './utils';

const {eye, clone, format, add, subtract, inv, transpose, matrix} = require('mathjs');
const {multiply} = require('./utils');

type Params = {
  MeasurementCovariance: Matrix;
  ProcessCovariance: Matrix;
  observationFunct: (x: Matrix, t: number) => Matrix;
  observationJacobianFunct: (x: Matrix, t: number) => Matrix;
  stateTransitionFunct: (x: Matrix, u: Matrix, t: number) => Matrix;
  stateTransitionJacobianFunct: (x: Matrix, u: Matrix, t: number) => Matrix;
};

class ExtendedKalmanFilter {
  State: Matrix;
  StateCovariance: Matrix;
  params: Params;
  NUM_DIMENSIONS: number;

  constructor(
    {InitialCovariance, InitialState, params}: {
      InitialCovariance: Matrix;
      InitialState: Matrix;
      params: Params;
    }
  ) {
    this.State = InitialState;
    this.StateCovariance = InitialCovariance;
    this.params = params;
    this.NUM_DIMENSIONS = InitialState.size()[0];
  }

  step(
    MeasurementInput: Matrix,
    ControlInput: Matrix,
    t: number
  ): {State: Matrix; StateCovariance: Matrix} {
    let {
      State: x,
      StateCovariance: P,
      params: {
        MeasurementCovariance: R,
        ProcessCovariance: Q,
        observationFunct: h,
        observationJacobianFunct,
        stateTransitionFunct: f,
        stateTransitionJacobianFunct,
      },
      NUM_DIMENSIONS: N,
    } = this;

    let u = ControlInput;
    let z = MeasurementInput;
    let F = stateTransitionJacobianFunct(x._data, u._data, t);
    let H = observationJacobianFunct(x._data, t);

    // predict
    let xPriori = f(x._data, u._data, t);
    let PPriori = add(Q, multiply(F, P, transpose(F)));

    // measurement and innovation
    let y = subtract(z, h(xPriori._data, t));
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
