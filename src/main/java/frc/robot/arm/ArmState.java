package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import frc.robot.constants.RobotLimits;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmState implements LoggableInputs, Cloneable {
  // Set of Preset ArmStates
  public static final ArmState IDLE = new ArmState(Math.PI / 2.0, RobotLimits.kMinArmLengthMeters);

  public static final ArmState SCORE_HYBRID = new ArmState(-0.3573930726493018, 0.52705);
  public static final ArmState SCORE_MID_CUBE = new ArmState(0.3205280129168529, 0.52705);
  public static final ArmState SCORE_HIGH_CUBE = new ArmState(0.432861251376473, 1.0498237311755916);
  public static final ArmState SCORE_MID_CONE = new ArmState(0.5450884182598523, 0.9092489998705285);
  public static final ArmState SINGLE_SUBSTATION = new ArmState(0.423389123941221, 0.52705);
  public static final ArmState DOUBLE_SUBSTATION = new ArmState(0.4885123780965301, 1.1938);

  public double AngleRadians;
  public double VelocityRadiansPerSecond;

  public double PivotToEffectorDistanceMeters;

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
  }

  /**
   * Return a copy of the ArmState with the angle reflected across the y-axis.
   *
   * @return reflected angle.
   */
  public ArmState inverted() {
    return new ArmState(Math.PI - AngleRadians, PivotToEffectorDistanceMeters);
  }

  /**
   * Return the ArmState as a vector.
   *
   * @return ArmState as a vector.
   */
  public Vector<N2> toVec() {
    return VecBuilder.fill(AngleRadians, PivotToEffectorDistanceMeters);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("AngleRadians", AngleRadians);
    table.put("LengthMeters", PivotToEffectorDistanceMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    AngleRadians = table.getDouble("AngleRadians", AngleRadians);
    PivotToEffectorDistanceMeters =
        table.getDouble("ExtensionLengthMeters", PivotToEffectorDistanceMeters);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(AngleRadians - other.AngleRadians) < Math.toRadians(0.25)
          && Math.abs(PivotToEffectorDistanceMeters - other.PivotToEffectorDistanceMeters) < 5e-3;
    }
    return false;
  }

  @Override
  public ArmState clone() throws CloneNotSupportedException {
    ArmState clone = (ArmState) super.clone();
    clone.AngleRadians = AngleRadians;
    clone.PivotToEffectorDistanceMeters = PivotToEffectorDistanceMeters;

    return clone;
  }
}
