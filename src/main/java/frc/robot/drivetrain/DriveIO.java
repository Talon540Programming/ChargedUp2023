package frc.robot.drivetrain;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent devices used in a Differential Drivetrain. */
public interface DriveIO extends LoggedIO<DriveIO.DriveIOInputs> {
  @AutoLog
  class DriveIOInputs {
    public double LeftPositionMeters = 0.0;
    public double LeftVelocityMetersPerSecond = 0.0;
    public double RightPositionMeters = 0.0;
    public double RightVelocityMetersPerSecond = 0.0;
  }

  @Override
  default void updateInputs(DriveIOInputs inputs) {}

  /**
   * Drive the robot based on output voltage to set to each side of the drivetrain. This is useful
   * when controlling the drivetrain using closed loop control via Feed Forward control.
   *
   * @param leftVolts output voltage for the left side of the drivetrain.
   * @param rightVolts output voltage for the right side of the drivetrain.
   */
  default void setVoltage(double leftVolts, double rightVolts) {}

  /**
   * Set the neutral mode of the drivetrain motors.
   *
   * @param mode Neutral mode to set the drivetrain motors to.
   */
  default void setNeutralMode(DriveNeutralMode mode) {}

  /** Reset the values of the drivetrain encoders. */
  default void resetEncoders() {}

  enum DriveNeutralMode {
    BRAKE,
    COAST
  }
}
