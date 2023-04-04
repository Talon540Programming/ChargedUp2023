package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.RobotLimits;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmState {
  // Set of Preset ArmStates
  public static final ArmState IDLE = new ArmState(Math.PI / 2.0, RobotLimits.kMinArmLengthMeters);

  public static final ArmState SCORE_HYBRID = new ArmState(-0.357393, 0.52705);
  public static final ArmState SCORE_MID_CUBE = new ArmState(0.32053, 0.52705);
  public static final ArmState SCORE_HIGH_CUBE = new ArmState(0.43286, 1.04982);
  public static final ArmState SCORE_MID_CONE = new ArmState(0.54509, 0.90925);
  public static final ArmState SINGLE_SUBSTATION = new ArmState(0.42339, 0.52705);
  public static final ArmState DOUBLE_SUBSTATION = new ArmState(0.48851, 1.19380);

  public final double AngleRadians;
  public final double VelocityRadiansPerSecond;
  public final double PivotToEffectorDistanceMeters;

  /**
   * Create an ArmState object.
   *
   * @param angleRad angle of the arm in radians.
   * @param armVelocityRadPerSecond velocity of the arm in radians per second.
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   */
  public ArmState(double angleRad, double armVelocityRadPerSecond, double pivotToEffectorMeters) {
    this.AngleRadians = angleRad;
    this.VelocityRadiansPerSecond = armVelocityRadPerSecond;
    this.PivotToEffectorDistanceMeters = pivotToEffectorMeters;
  }

  /**
   * Create an ArmState object. Assumed the arm shouldn't be moving (velocity of 0).
   *
   * @param angleRad angle of the arm in radians.
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   */
  public ArmState(double angleRad, double pivotToEffectorMeters) {
    this.AngleRadians = angleRad;
    this.PivotToEffectorDistanceMeters =
        MathUtil.clamp(
            pivotToEffectorMeters,
            RobotLimits.kMinArmLengthMeters,
            RobotLimits.kMaxArmLengthMeters);
    this.VelocityRadiansPerSecond = 0;
  }

  /**
   * Return a copy of the ArmState with the angle reflected across the y-axis.
   *
   * @return reflected angle.
   */
  public ArmState invert() {
    return new ArmState(Math.PI - AngleRadians, PivotToEffectorDistanceMeters);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(AngleRadians - other.AngleRadians) < 0.05
          && Math.abs(PivotToEffectorDistanceMeters - other.PivotToEffectorDistanceMeters) < 0.01;
    }
    return false;
  }
}
