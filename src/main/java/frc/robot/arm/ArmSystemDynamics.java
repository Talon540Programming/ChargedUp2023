package frc.robot.arm;

import edu.wpi.first.math.util.Units;

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

  public static double calculateRotationFeedForward(ArmState currentState, ArmState setpointState) {
    return  0.0;
  }
}
