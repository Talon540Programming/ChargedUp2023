package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.RobotLimits;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmState implements LoggableInputs, Cloneable {
  public static final ArmState kDefaultState = new ArmState(Math.PI / 2.0, RobotLimits.kMinArmLengthMeters);

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
   * Convert the ArmState to a Trapezoidal Motion Profile Position State as a {@link
   * TrapezoidProfile.State}.
   *
   * @return ArmState as a position state.
   */
  public TrapezoidProfile.State getRotationState() {
    return new TrapezoidProfile.State(AngleRadians, 0);
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
