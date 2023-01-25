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

    public Rotation2d getRotation2d();

    public double getYaw();
    public double getPitch();
    public double getRoll();

    public void resetHeading();

    public default boolean isLevel() {
        return Math.abs(getPitch()) < Math.toRadians(Constants.Drivetrain.kRobotStabilizationToleranceDegrees);
    }

    public default Rotation3d getRotation3d() {
        return new Rotation3d(getRoll(), getPitch(), getYaw());
    }

}
