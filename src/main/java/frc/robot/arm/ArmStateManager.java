package frc.robot.arm;

import frc.lib.arm.ArmUtil;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

public class ArmStateManager {
  /** Singleton instance of the ArmStateManager. */
  private static ArmStateManager instance;

  /**
   * Get the instance of the ArmStateManager Singleton.
   *
   * @return instance of ArmStateManager.
   */
  public static synchronized ArmStateManager getInstance() {
    if (instance == null) {
      instance = new ArmStateManager();
    }
    return instance;
  }

  /** The default (neutral) mode for the arm. */
  public static final ArmState kDefaultState = new ArmState(Math.PI / 2.0, 0);

  private ArmState armState = kDefaultState;

  private double robotPitchRadians = 0;
  private boolean balanceModeEnabled = false;

  /**
   * Update the ArmState of the state manager. This method will check if the state is breaking
   * extension rules or would cause the robot to tip. If it is unable to fix a State, it will not
   * change the current state. If balance mode is enabled, it will use the arm as a counterweight
   * in-order to help balance the robot.
   *
   * @param state State to set.
   */
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
      if (RobotLimits.kRearLimit <= state.AngleRadians
          && state.AngleRadians <= RobotLimits.kForwardLimitRadians) {
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

  /**
   * Get the target state of the Arm.
   *
   * @return target state.
   */
  public ArmState getArmState() {
    return armState;
  }

  /**
   * Set the pitch of the robot in radians.
   *
   * @param angleRadians pitch of the robot in radians.
   */
  public void updateRobotPitch(double angleRadians) {
    robotPitchRadians = angleRadians;
    updateArmState(armState);
  }

  /**
   * Set if the StateManager should update the position of arm in order to help balance the robot.
   *
   * @param enable Whether to enable balance mode.
   */
  public void enableBalanceMode(boolean enable) {
    balanceModeEnabled = enable;
    updateArmState(armState);
  }

  /** Reset the ArmState to the Neutral State of the arm {@link ArmStateManager#kDefaultState}. */
  public void resetNeutralState() {
    armState = kDefaultState;
  }
}
