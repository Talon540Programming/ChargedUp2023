package frc.robot.intake;

import frc.robot.constants.Constants;

public class IntakeStateManager {
  private static IntakeStateManager instance;

  public static synchronized IntakeStateManager getInstance() {
    if (instance == null) {
      instance = new IntakeStateManager();
    }
    return instance;
  }

  private IntakeState state;

  private ValidClawState clawState;
  private ValidWristState wristState;

  public void updateState(ValidClawState clawState) {
    updateState(clawState, wristState);
  }

  public void updateState(ValidWristState wristState) {
    updateState(clawState, wristState);
  }

  public void updateState(ValidClawState clawState, ValidWristState wristState) {
    this.clawState = clawState;
    this.wristState = wristState;

    this.state = new IntakeState(clawState.angle, wristState.angle);
  }

  public IntakeState getTargetState() {
    return state;
  }

  public ValidClawState getTargetClawState() {
    return clawState;
  }

  public ValidWristState getTargetWristState() {
    return wristState;
  }

  public void flipWrist() {
    updateState(
        wristState == ValidWristState.Idle ? ValidWristState.Flipped : ValidWristState.Idle);
  }

  public enum ValidClawState {
    Idle(Constants.Intake.kIntakeClawMaximumAngleRad),
    HoldingCone(Constants.Intake.kConeIntakeAngle),
    HoldingCube(Constants.Intake.kCubeIntakeAngle);

    public final double angle;

    ValidClawState(double angle) {
      this.angle = angle;
    }
  }

  public enum ValidWristState {
    Idle(Constants.Intake.kWristIdleAngleRad),
    Flipped(Constants.Intake.kWristFlippedAngleRad);

    public final double angle;

    ValidWristState(double angle) {
      this.angle = angle;
    }
  }
}
