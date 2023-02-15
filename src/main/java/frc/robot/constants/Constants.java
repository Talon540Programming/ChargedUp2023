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

    public static final DriveNeutralMode kDrivetrainDefaultNeutralMode = DriveNeutralMode.COAST;

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

    // First extrusion is from fulcrum, not end of extrusion
    public static final double kFirstExtrusionLengthInches = 17; // TODO
    public static final double kFirstExtrusionLengthMeters = Units.inchesToMeters(kFirstExtrusionLengthInches);

    // Second and Third extrusion lengths are without the end caps
    // which are retained within their encapsulating extrusions.
    public static final double kSecondExtrusionLengthInches = 25; // TODO
    public static final double kSecondExtrusionLengthMeters = Units.inchesToMeters(kSecondExtrusionLengthInches);

    public static final double kThirdExtrusionLengthInches = 18; // TODO
    public static final double kThirdExtrusionLengthMeters = Units.inchesToMeters(kThirdExtrusionLengthInches);
  }

  public static final class Grabber {
    // This includes the mounting hardware onto the third extrusion which is why it lacks it.
    public static final double kLengthInches = 17; // TODO
    public static final double kLengthMeters = Units.inchesToMeters(kLengthInches);
  }
}
