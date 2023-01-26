package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.drivetrain.DriveIO.DriveNeutralMode;

public final class Constants {
  private static final RobotType kRobotType = RobotType.ROBOT_2023P;

  public enum RobotMode {
    REAL,
    REPLAY
  }

  public enum RobotType {
    ROBOT_2023P,
    ROBOT_2023C,
    ROBOT_2022C,
    ROBOT_2020C
  }

  public static RobotType getRobotType() {
    return kRobotType;
  }

  @SuppressWarnings("UnnecessaryDefault")
  public static RobotMode getRobotMode() {
    return switch (getRobotType()) {
      case ROBOT_2020C, ROBOT_2022C, ROBOT_2023P, ROBOT_2023C -> RobotBase.isReal()
          ? RobotMode.REAL
          : RobotMode.REPLAY;
      default -> RobotMode.REAL;
    };
  }

  public static final class Drivetrain {
    public static final double kMaxDrivetrainVelocityMetersPerSecond = 4;
    public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxDrivetrainRotationalVelocityRadiansPerSecond = 0; // TODO
    public static final double kMaxDrivetrainRotationalAccelerationRadiansPerSecondSquared =
        0; // TODO

    public static final double kDrivetrainGearRatio = 11.78;

    public static final double kTrackWidthInches = 19.618320; // TODO test in SysID
    public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

    public static final double kWheelRadiusInches = 3;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

    public static final DifferentialDriveKinematics kDrivetrainKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final DriveNeutralMode kDrivetrainDefaultNeutralMode = DriveNeutralMode.COAST;

    public static final double kRobotStabilizationToleranceDegrees = 1;

    public static class ControlValues {
      public static class WheelSpeed {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO

        public static final double kS = 0; // TODO
        public static final double kV = 0; // TODO
        public static final double kA = 0; // TODO
      }

      public static class Trajectory {
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
      }

      public static class Stabilization {
        public static final double kP = 1.0 / 30.0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }
}
