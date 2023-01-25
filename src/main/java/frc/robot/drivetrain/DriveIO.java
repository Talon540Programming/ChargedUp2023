package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Flags.NeutralMode;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double LeftPosition = 0.0;
    public double LeftVelocity = 0.0;
    public double RightPosition = 0.0;
    public double RightVelocity = 0.0;
    public double GyroYaw = 0.0;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void setPercent(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1, 1);

    setVoltage(12.0 * leftPercent, 12.0 * rightPercent);
  }

  public default void setNeutralMode(NeutralMode mode) {}

  public default Rotation2d getHeading() {
    return new Rotation2d();
  }

  public default void resetEncoders() {}

  public default void zeroHeading() {}

  public default double getLeftPositionMeters() {
    return 0;
  }

  public default double getLeftVelocityMetersPerSecond() {
    return 0;
  }

  public default double getRightPositionMeters() {
    return 0;
  }

  public default double getRightVelocityMetersPerSecond() {
    return 0;
  }

  public default double getRate() {
    return 0;
  }
}
