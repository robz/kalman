/* @flow */

type Matrix = Object;

const {eye, clone, format, add, subtract, inv, transpose, matrix} = require('mathjs');
const {multiply} = require('./utils');

class KalmanFilter {
  ControlModel: Matrix;
  MeasurementCovariance: Matrix;
  ObservationModel: Matrix;
  ProcessCovariance: Matrix;
  State: Matrix;
  StateCovariance: Matrix;
  StateTransitionModel: Matrix;
  NUM_DIMENSIONS: number;

  constructor({
    ControlModel,
    InitialCovariance,
    InitialState,
    MeasurementCovariance,
    ObservationModel,
    ProcessCovariance,
    StateTransitionModel,
  }: Object) {
    this.ControlModel = ControlModel;
    this.MeasurementCovariance = MeasurementCovariance;
    this.ObservationModel = ObservationModel;
    this.ProcessCovariance = ProcessCovariance;
    this.State = InitialState;
    this.StateCovariance = InitialCovariance;
    this.StateTransitionModel = StateTransitionModel;

    this.NUM_DIMENSIONS = InitialState.size()[0];
  }

  step(
    MeasurementInput: Matrix,
    ControlInput: Matrix,
    ProcessCovariance?: Matrix
  ): {State: Matrix; StateCovariance: Matrix} {
    let {
      ControlModel: B,
      MeasurementCovariance: R,
      ObservationModel: H,
      ProcessCovariance: Q,
      State: x,
      StateCovariance: P,
      StateTransitionModel: F,
      NUM_DIMENSIONS: N,
    } = this;

    let u = ControlInput;
    let z = MeasurementInput;

    if (ProcessCovariance) {
      Q = ProcessCovariance;
    }

    // predict
    let xPriori = add(multiply(F, x), multiply(B, u));
    let PPriori = add(Q, multiply(F, P, transpose(F)));

    // measurement and innovation
    let y = subtract(z, multiply(H, xPriori));
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

module.exports = KalmanFilter;
