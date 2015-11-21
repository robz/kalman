/* @flow */

const {eye, clone, format, add, subtract, multiply, inv, transpose, matrix} = require('mathjs');

class KalmanFilter {
  constructor({
    ControlModel,
    InitialCovariance,
    InitialState,
    MeasurementCovariance,
    ObservationModel,
    ProcessCovariance,
    StateTransitionModelFunct,
  }) {
    this.ControlModel = ControlModel;
    this.PreviousCovariance = InitialCovariance;
    this.PreviousState = InitialState;
    this.MeasurementCovariance = MeasurementCovariance;
    this.ObservationModel = ObservationModel;
    this.ProcessCovariance = ProcessCovariance;
    this.StateTransitionModelFunct = StateTransitionModelFunct;

    this.NUM_DIMENSIONS = InitialState.size()[0];
  }

  step(MeasurementInput: Object, ControlInput: Object, dt: number): void {
    let {
      ControlModel,
      PreviousCovariance,
      PreviousState,
      MeasurementCovariance,
      ObservationModel,
      ProcessCovariance,
      StateTransitionModelFunct,
      NUM_DIMENSIONS,
    } = this;

    let StateTransitionModel = StateTransitionModelFunct(dt);

    let StatePrediction = add(
      multiply(StateTransitionModel, PreviousState),
      multiply(ControlModel, ControlInput)
    );

    let CovariancePrediction = add(
      multiply(
        StateTransitionModel,
        multiply(
          PreviousCovariance,
          transpose(StateTransitionModel)
        )
      ),
      ProcessCovariance
    );

    let Innovation = subtract(
      MeasurementInput,
      multiply(ObservationModel, StatePrediction)
    );

    let InnovationCovariance = add(
      multiply(
        ObservationModel,
        multiply(
          CovariancePrediction,
          transpose(ObservationModel)
        )
      ),
      MeasurementCovariance
    );

    let KalmanGain = multiply(
      CovariancePrediction,
      multiply(
        transpose(ObservationModel),
        inv(InnovationCovariance)
      )
    );

    let NextState = add(
      StatePrediction,
      multiply(KalmanGain, Innovation)
    );

    let NextCovariance = multiply(
      subtract(
        eye(NUM_DIMENSIONS),
        multiply(KalmanGain, ObservationModel)
      ),
      CovariancePrediction
    );

    this.PreviousState = clone(NextState);
    this.PreviousCovariance = clone(NextCovariance);
  }
}

module.exports = KalmanFilter;
