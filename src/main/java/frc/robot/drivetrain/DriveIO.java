package frc.robot.drivetrain;

import frc.robot.constants.Flags.NeutralMode;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
public interface DriveIO extends LoggedIO<DriveIO.DriveIOInputs> {
  @AutoLog
  public static class DriveIOInputs {
    public double LeftPositionMeters = 0.0;
    public double LeftVelocityMetersPerSecond = 0.0;
    public double RightPositionMeters = 0.0;
    public double RightVelocityMetersPerSecond = 0.0;

    public double GyroYawRad = 0.0;
    public double GyroPitchRad = 0.0;
    public double GyroRollRad = 0.0;
    public double GyroRateRadPerSecond = 0.0;
  }

  /**
   * Update all loggable inputs.
   *
   * @param inputs inputs to update.
   */
  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void setNeutralMode(NeutralMode mode) {}

  public default void resetEncoders() {}

  public default void zeroHeading() {}
}
