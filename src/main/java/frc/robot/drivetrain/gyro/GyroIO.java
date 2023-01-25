package frc.robot.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO extends LoggedIO<GyroIO.GyroIOInputs> {
  @AutoLog
  public static class GyroIOInputs {
    public double GyroYawRad = 0.0;
    public double GyroPitchRad = 0.0;
    public double GyroRollRad = 0.0;
    public double GyroRateRadPerSecond = 0.0;
  }

  @Override
  public default void updateInputs(GyroIOInputs inputs) {}
  ;

  public default Rotation2d getRotation2d() {
    return new Rotation2d();
  }
  ;

  public default double getYaw() {
    return 0;
  }
  ;

  public default double getPitch() {
    return 0;
  }
  ;

  public default double getRoll() {
    return 0;
  }
  ;

  public default void resetHeading() {}
  ;

  public default boolean isLevel() {
    return Math.abs(getPitch())
        < Math.toRadians(Constants.Drivetrain.kRobotStabilizationToleranceDegrees);
  }

  public default Rotation3d getRotation3d() {
    return new Rotation3d(getRoll(), getPitch(), getYaw());
  }
}
