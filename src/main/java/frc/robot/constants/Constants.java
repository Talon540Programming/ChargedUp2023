package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class Drivetrain {
    public static final double kMaxDrivetrainVelocityMetersPerSecond = 0; // TODO
    public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared = 0; // TODO
    public static final double kMaxDrivetrainRotationalVelocityRadiansPerSecond = 0; // TODO

    public static final double kDrivetrainGearRatio = 54.0 / 20.0; // TODO

    public static final double kTrackWidthMeters = 0; // TODO
    public static final double kWheelRadiusInches = 3;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

    public static final DifferentialDriveKinematics kDrivetrainKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final NeutralMode kDrivetrainNeutralMode = NeutralMode.Brake;
  }
}
