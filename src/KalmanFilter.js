/* @flow */

type Matrix = Object;

const {eye, clone, format, add, subtract, multiply: multiplyTwo, inv, transpose, matrix} = require('mathjs');

// matrix multiply with any number of matricies
function multiply(...args): Matrix {
  if (args.length < 2) {
    throw new Error('multiply must take at least 2 arguments');
  }

  let result = multiplyTwo(args[0], args[1]);
  for (let i = 2; i < args.length; i++) {
    // matrix multiplication is associative, so A*B*C = (A*B)*C
    result = multiplyTwo(result, args[i]);
  }

  return result;
}

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
    ControlInput: Matrix
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
