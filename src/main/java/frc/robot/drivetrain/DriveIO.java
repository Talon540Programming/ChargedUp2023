package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants.NeutralMode;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent devices used in a Differential Drivetrain. */
public interface DriveIO extends LoggedIO<DriveIO.DriveIOInputs> {
  @AutoLog
  class DriveIOInputs {
    public double LeftPositionMeters;
    public double LeftVelocityMetersPerSecond;
    public double RightPositionMeters;
    public double RightVelocityMetersPerSecond;
    public double[] TemperatureCelsius = new double[] {};
    public double[] SupplyCurrentAmps = new double[] {};

    public boolean GyroConnected;

    public double YawPositionRad;
    public double PitchPositionRad;
    public double RollPositionRad;

    public double YawRateRadPerSecond;
    public double PitchRateRadPerSecond;
    public double RollRateRadPerSecond;

    public double AccelXGForces;
    public double AccelYGForces;
    public double AccelZGForces;
  }

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
  default void setNeutralMode(NeutralMode mode) {}

  /** Reset the values of the drivetrain encoders. */
  default void resetEncoders() {}

  /** Reset the yaw of the gyro to 0. */
  default void resetHeading() {}

  /**
   * Get the heading of the robot as a {@link Rotation2d} object.
   *
   * @return robot heading.
   */
  default Rotation2d getHeading() {
    return new Rotation2d();
  }
}
