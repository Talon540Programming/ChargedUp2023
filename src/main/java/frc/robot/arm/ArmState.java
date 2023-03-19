package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.RobotLimits;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmState implements LoggableInputs, Cloneable {
  // Set of Preset ArmStates
  public static final ArmState IDLE = new ArmState(Math.PI / 2.0, RobotLimits.kMinArmLengthMeters);
  public static final ArmState SCORE_HYBRID = null;
  public static final ArmState SCORE_MID_CUBE = null;
  public static final ArmState SCORE_HIGH_CUBE = null;
  public static final ArmState SCORE_MID_CONE = null;
  public static final ArmState SCORE_HIGH_CONE = null;
  public static final ArmState SINGLE_SUBSTATION = null;
  public static final ArmState DOUBLE_SUBSTATION = null;
  public static final ArmState FLOOR = null;

  public double AngleRadians;
  public double LengthMeters;

  /**
   * Create an ArmState object.
   *
   * @param angleRadians angle of the arm in radians.
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the effector.
   */
  public ArmState(double angleRadians, double pivotToEffectorMeters) {
    this.AngleRadians = angleRadians;
    this.LengthMeters = pivotToEffectorMeters;
  }

  /**
   * Return a copy of the ArmState with the angle reflected across the y-axis.
   * @return reflected angle.
   */
  public ArmState inverted() {
    return new ArmState(Math.PI - AngleRadians, LengthMeters) ;
  }

  /**
   * Return the ArmState as a vector.
   * @return ArmState as a vector.
   */
  public Vector<N2> toVec() {
    return VecBuilder.fill(AngleRadians, LengthMeters);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("AngleRadians", AngleRadians);
    table.put("LengthMeters", LengthMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    AngleRadians = table.getDouble("AngleRadians", AngleRadians);
    LengthMeters = table.getDouble("ExtensionLengthMeters", LengthMeters);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(AngleRadians - other.AngleRadians) < Math.toRadians(0.25)
          && Math.abs(LengthMeters - other.LengthMeters) < 5e-3;
    }
    return false;
  }

  @Override
  public ArmState clone() throws CloneNotSupportedException {
    ArmState clone = (ArmState) super.clone();
    clone.AngleRadians = AngleRadians;
    clone.LengthMeters = LengthMeters;

    return clone;
  }
}
