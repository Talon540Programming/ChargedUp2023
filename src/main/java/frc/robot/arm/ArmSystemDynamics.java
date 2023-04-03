package frc.robot.arm;

public class ArmSystemDynamics {
  private ArmSystemDynamics() {}

  /**
   * Calculate the Moment of Inertia of the entire arm system based on the length from the pivot
   * point to the beginning of the effector. Formula is estimated using Polynomial regression found
   * <a href="https://www.desmos.com/calculator/1ok9sw0qvz">here</a>. Regression follows an R^2
   * value of 0.9999.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated MoI of the entire Arm System in kilograms meters squared.
   */
  public static double calculateSystemMomentOfInertia(double pivotToEffectorMeters) {
    return 2.63653 * Math.pow(pivotToEffectorMeters, 2)
        + 2.29501 * pivotToEffectorMeters
        - 0.153263;
  }

  /**
   * Calculate the FeedForward value to apply in volts. Doesn't take acceleration into account.
   *
   * @param setpointState the setpoint to calculate FeedForward values for.
   * @return feedforward value in volts.
   */
  public static double calculateRotationFeedForward(ArmState setpointState) {
    double kS = 1; // TODO find this using sysID
    double kG =
        1; // TODO find this using Linear or Polynomial Regression over multiple runs of SysID
    double kV = 0.5; // TODO find this using sysID
    double kA =
        0.1; // TODO find this using Linear or Polynomial Regression over multiple runs of SysID

    return kS * Math.signum(setpointState.VelocityRadiansPerSecond)
        + kG * Math.cos(setpointState.AngleRadians)
        + kV * setpointState.VelocityRadiansPerSecond;
  }
}
