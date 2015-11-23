/* @flow */

const {eye, clone, format, add, subtract, multiply, inv, transpose, matrix} = require('mathjs');

type Matrix = Object;

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
    this.StateCovariance = InitialCovariance;
    this.State = InitialState;
    this.MeasurementCovariance = MeasurementCovariance;
    this.ObservationModel = ObservationModel;
    this.ProcessCovariance = ProcessCovariance;
    this.StateTransitionModel = StateTransitionModel;

    this.NUM_DIMENSIONS = InitialState.size()[0];
  }

  step(MeasurementInput: Object, ControlInput: Object): void {
    let {
      ControlModel: B,
      StateCovariance: P,
      State: x,
      MeasurementCovariance: R,
      ObservationModel: H,
      ProcessCovariance: Q,
      StateTransitionModel: F,
      NUM_DIMENSIONS: N,
    } = this;

    let u = ControlInput;
    let z = MeasurementInput;

    let xPriori = add(
      multiply(F, x),
      multiply(B, u)
    );

    let PPriori = add(
      multiply(
        F,
        multiply(
          P,
          transpose(F)
        )
      ),
      Q
    );

    let y = subtract(
      z,
      multiply(H, xPriori)
    );

    let S = add(
      multiply(
        H,
        multiply(
          PPriori,
          transpose(H)
        )
      ),
      R
    );

    let K = multiply(
      PPriori,
      multiply(
        transpose(H),
        inv(S)
      )
    );

    let xPosteriori = add(
      xPriori,
      multiply(K, y)
    );

    let PPosteriori = multiply(
      subtract(
        eye(N),
        multiply(K, H)
      ),
      PPriori
    );

    this.State = xPosteriori;
    this.StateCovariance = PPosteriori;
  }
}

module.exports = KalmanFilter;
