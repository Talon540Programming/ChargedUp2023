package frc.robot.constants;

public final class Flags {
  public enum RobotMode {
    /** Running on the proto robot. */
    PROTO,
    /** Running on the comp robot. */
    COMP,
    /** Replaying from a log file. */
    REPLAY
  }

  public enum NeutralMode {
    BRAKE,
    COAST
  }
}
