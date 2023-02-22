package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private static final RobotType kRobotType = RobotType.ROBOT_2023P;

  public enum RobotMode {
    REAL,
    REPLAY
  }

  public enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P
  }

  public static RobotType getRobotType() {
    return kRobotType;
  }

  @SuppressWarnings("UnnecessaryDefault")
  public static RobotMode getRobotMode() {
    return switch (getRobotType()) {
      case ROBOT_2023C, ROBOT_2023P -> RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
      default -> RobotMode.REAL;
    };
  }

  public static final class Drivetrain {
    public static final boolean kLeftSideInverted = true;
    public static final boolean kLeftSensorInverted = false;
    public static final boolean kRightSideInverted = false;
    public static final boolean kRightSensorInverted = true;

    public static final double kMaxDrivetrainVelocityMetersPerSecond = 4;
    public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared = 3.5;

    public static final double kDrivetrainGearRatio = (54.0 / 20.0) * (50.0 / 11.0);

    public static final double kTrackWidthInches = 19.618320; // TODO test in SysID
    public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

    public static final double kWheelRadiusInches = 3;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

    public static final DifferentialDriveKinematics kDrivetrainKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final NeutralMode kDrivetrainDefaultNeutralMode = NeutralMode.COAST;

    public static final double kRobotStabilizationToleranceDegrees = 1; // TODO

    public static class ControlValues {
      public static class WheelSpeed {
        public static final double kP = 0.47934; // TODO
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.077705; // TODO
        public static final double kV = 2.8428; // TODO
        public static final double kA = 0.10828; // TODO
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

  public static final class Arm {
    public static final boolean kRotationInverted = false;
    public static final boolean kExtensionInverted = false;
    public static final boolean kExtensionEncoderInverted = false;

    public static final double kExtensionGearRatio = 4.0 / 1.0;
    public static final double kExtensionWinchRadiusInches = 0.4;
    public static final double kExtensionWinchRadiusMeters =
        Units.inchesToMeters(kExtensionWinchRadiusInches);

    public static final double kExtensionPositionConversionFactor =
        Math.PI * (kExtensionWinchRadiusMeters / kExtensionGearRatio); // TODO
    public static final double kExtensionVelocityConversionFactor =
        Math.PI / (30.0 * kExtensionGearRatio); // TODO

    public static class ControlValues {
      public static class RotationValues {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO

        public static final double kS = 0; // TODO
        public static final double kG = 0; // TODO
        public static final double kV = 0; // TODO
        public static final double kA = 0; // TODO
      }

      public static class ExtensionValues {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }

  public enum NeutralMode {
    BRAKE,
    COAST
  }
}
