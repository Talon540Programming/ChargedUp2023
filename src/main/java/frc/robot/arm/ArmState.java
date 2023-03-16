package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** An object that can be used to track or represent the state of, or desired state of the arm. */
public class ArmState implements LoggableInputs, Cloneable {
  public double AngleRadians;
  public double ArmLengthMeters;

  /**
   * Create an ArmState object.
   *
   * @param angleRadians Angle of the Arm in radians. This angle should be similar to the Unit
   *     circle where 0 means the arm is perpendicular to the robot's fulcrum and parallel to the
   *     floor, facing towards the front of the robot. The angle is CCW positive.
   * @param armLengthMeters The length that the arm is extended to beyond the fully retracted
   *     state at without the grabber or any other * attachment at the end in meters.
   */
  public ArmState(double angleRadians, double armLengthMeters) {
    this.AngleRadians = angleRadians;
    this.ArmLengthMeters = armLengthMeters;
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
    table.put("ArmLengthMeters",
            ArmLengthMeters
    );
  }

  @Override
  public void fromLog(LogTable table) {
    AngleRadians = table.getDouble("AngleRadians", AngleRadians);
    ArmLengthMeters = table.getDouble("ExtensionLengthMeters",
            ArmLengthMeters
    );
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(AngleRadians - other.AngleRadians) < Math.toRadians(0.25)
          && Math.abs(ArmLengthMeters - other.ArmLengthMeters) < 5e-3;
    }
    return false;
  }

  @Override
  public ArmState clone() {
    return new ArmState(AngleRadians,
            ArmLengthMeters
    );
  }
}
