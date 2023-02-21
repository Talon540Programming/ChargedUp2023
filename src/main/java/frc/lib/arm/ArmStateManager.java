package frc.lib.arm;

import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

public class ArmStateManager {
  private static ArmStateManager instance;

  public static synchronized ArmStateManager getInstance() {
    if (instance == null) {
      instance = new ArmStateManager();
    }
    return instance;
  }

  public static final ArmState kDefaultState = new ArmState(Math.PI / 2.0, 0);

  private ArmState armState = kDefaultState;

  private double robotPitchRadians = 0;
  private boolean balanceModeEnabled = false;

  public void updateArmState(ArmState state) {
    // Organize the state around the current robot state
    if (balanceModeEnabled) {
      state.ExtensionLengthMeters = 0;
      state.AngleRadians = (Math.PI / 2) - robotPitchRadians;
    } else {
      // Normalize the state
      state.AngleRadians %= Math.PI * 2.0;
      state.ExtensionLengthMeters = Math.max(state.ExtensionLengthMeters, 0);

      // Make sure the arm won't phase through the floor or drivetrain?
      if (RobotLimits.kRearLimit < state.AngleRadians
          && state.AngleRadians < RobotLimits.kForwardLimitRadians) {
        // The new target position is considered "incorrect" so we reject it, don't change the
        // current arm state.
        // TODO, do we want to reject the state, or try and fix it?
        return;
      }

      double cosTheta = Math.cos(state.AngleRadians);
      double sinTheta = Math.sin(state.AngleRadians);

      // Check the horizontal extension
      double horizontalDelta =
          cosTheta
              * (ArmUtil.calculateArmLength(state.ExtensionLengthMeters)
                  + RobotDimensions.Grabber.kLengthMeters);
      double horizontalDistanceFromFrame =
          RobotDimensions.Drivetrain.kDrivetrainLengthMeters / 2
              - horizontalDelta; // Divide by 2 because the fulcrum is in the center of the robot.

      if (horizontalDistanceFromFrame > RobotLimits.kMaxExtensionHorizontalMeters) {
        state.ExtensionLengthMeters =
            ArmUtil.calculateExtensionLength(
                (RobotLimits.kMaxExtensionHorizontalMeters / cosTheta)
                    - RobotDimensions.Grabber.kLengthMeters);
      }

      double verticalDelta =
          sinTheta
              * (ArmUtil.calculateArmLength(state.ExtensionLengthMeters)
                  + RobotDimensions.Grabber.kLengthMeters);

      if (verticalDelta > RobotLimits.kMaxExtensionVerticalMeters) {
        state.ExtensionLengthMeters =
            ArmUtil.calculateExtensionLength(
                (RobotLimits.kMaxExtensionVerticalMeters / sinTheta)
                    - RobotDimensions.Grabber.kLengthMeters);
      }
    }

    // Set as the current state. Clone to untie from the original object.
    armState = state.clone();
  }

  public ArmState getArmState() {
    return armState;
  }

  public void updateRobotPitch(double angleRadians) {
    robotPitchRadians = angleRadians;
    updateArmState(armState);
  }

  public void enableBalanceMode(boolean enable) {
    balanceModeEnabled = enable;
    updateArmState(armState);
  }

  public void resetToHome() {
    armState = kDefaultState;
  }
}
