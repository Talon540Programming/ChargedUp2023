package frc.lib.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** An object that can be used to track or represent the state of, or desired state of the arm. */
public class ArmState implements LoggableInputs, Cloneable {
  public double AngleRadians;
  public double ExtensionLengthMeters;

  /**
   * Create an ArmState object.
   *
   * @param angleRadians Angle of the Arm in radians. This angle should be similar to the Unit
   *     circle where 0 means the arm is perpendicular to the robot's fulcrum and parallel to the
   *     floor, facing towards the front of the robot. The angle is CCW positive.
   * @param extensionLengthMeters The length that the arm is extended to beyond the fully retracted
   *     state at without the grabber or any other * attachment at the end in meters.
   */
  public ArmState(double angleRadians, double extensionLengthMeters) {
    this.AngleRadians = angleRadians;
    this.ExtensionLengthMeters = extensionLengthMeters;
  }

  @Override
  public void toLog(LogTable table) {
    table.put("AngleRadians", AngleRadians);
    table.put("ExtensionLengthMeters", ExtensionLengthMeters);
  }

  @Override
  public void fromLog(LogTable table) {
    AngleRadians = table.getDouble("AngleRadians", AngleRadians);
    ExtensionLengthMeters = table.getDouble("ExtensionLengthMeters", ExtensionLengthMeters);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(AngleRadians - other.AngleRadians) < Math.toRadians(0.25)
          && Math.abs(ExtensionLengthMeters - other.ExtensionLengthMeters) < 5e-3;
    }
    return false;
  }

  @Override
  public ArmState clone() {
    return new ArmState(AngleRadians, ExtensionLengthMeters);
  }
}
