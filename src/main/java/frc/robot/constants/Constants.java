package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.arm.ArmKinematics;
import java.io.IOException;

public final class Constants {
  /**
   * Whether advanced logging should be enabled. This can be disabled if there is too much going on.
   */
  public static final boolean kAdvancedLoggingEnabled = true;

  private static final RobotType kRobotType = RobotType.ROBOT_2023C;

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
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }

  public static final class Arm {
    public static final ArmKinematics kArmKinematics =
        new ArmKinematics(
            new Pose3d(
                0,
                0,
                Units.inchesToMeters(RobotDimensions.Arm.kFulcrumHeightInches),
                new Rotation3d()));
    public static final boolean kRotationInverted = false; // TODO
    public static final boolean kExtensionInverted = false; // TODO

    @SuppressWarnings("PointlessArithmeticExpression")
    public static final double kExtensionGearRatio = 4.0 / 1.0; // TODO

    public static final double kExtensionWinchRadiusInches = 0.4; // TODO
    public static final double kExtensionWinchRadiusMeters =
        Units.inchesToMeters(kExtensionWinchRadiusInches);

    public static final double kExtensionCableLengthInches = 0; // TODO;
    public static final double kExtensionCableLengthMeters =
        Units.inchesToMeters(kExtensionCableLengthInches);

    public static final double kExtensionCableDiameterInches = 0; // TODO
    public static final double kExtensionCableDiameterMeters =
        Units.inchesToMeters(kExtensionCableDiameterInches);

    public static final int kNumberOfWrapsPerRadiusIncrease = 0; // TODO
    public static final double kInitialWrapsAtBoot = 0; // TODO

    public static final double kExtensionPositionConversionFactor = (1 / kExtensionGearRatio);
    public static final double kExtensionVelocityConversionFactor =
        Math.PI / (30.0 * kExtensionGearRatio); // TODO

    public static final double kRotationAbsoluteEncoderOffsetDegrees = 0; // TODO

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

  public static final class Vision {
    public static final AprilTagFieldLayout kFieldLayout;

    static {
      try {
        kFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      } catch (IOException e) {
        throw new RuntimeException("Unable to Load the AprilTagFieldLayout of the field");
      }
    }

    public static final Transform3d kForwardCameraTransform3d = null; // TODO
    public static final Transform3d kRearCameraTransform3d = null; // TODO
  }

  public static final class Intake {
    public static final double kWristChangePercent = 0.2; // TODO
    public static final double kClawChangePercent = 0.2; // TODO

    public static final double kIntakeClawMinimumAngleRad = 0; // TODO
    public static final double kIntakeClawMaximumAngleRad = 0; // TODO

    public static final double kConeIntakeAngle = 0; // TODO
    public static final double kCubeIntakeAngle = 0; // TODO

    public static final double kWristPositionConversionFactor = 0; // TODO
    public static final double kWristVelocityConversionFactor = 0; // TODO

    public static final double kWristEncoderOffsetDegrees = 0; // TODO
    public static final double kClawEncoderOffsetDegrees = 0; // TODO

    public static final double kGamepeiceColorTolerance = 25; // TODO

    public static final double kWristIdleAngleRad = 0; // TODO
    public static final double kWristFlippedAngleRad = Math.PI; // TODO

    public static class ControlValues {
      public static class ClawPosition {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }

      public static class WristPosition {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }

  public enum NeutralMode {
    BRAKE,
    COAST;

    /**
     * Convert the Neutral mode to one used by the Phoenix APIs.
     *
     * @return Phoenix Neutral mode
     */
    public com.ctre.phoenix.motorcontrol.NeutralMode toPhoenixMode() {
      return switch (this) {
        case BRAKE -> com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
        case COAST -> com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
      };
    }

    /**
     * Convert the Neutral mode to one used by the SparkMax API.
     *
     * @return SparkMax Idle Mode.
     */
    public CANSparkMax.IdleMode toIdleMode() {
      return switch (this) {
        case BRAKE -> CANSparkMax.IdleMode.kBrake;
        case COAST -> CANSparkMax.IdleMode.kCoast;
      };
    }
  }

  public enum GamePiece {
    Cone(new Color8Bit(0, 0, 0)), // TODO
    Cube(new Color8Bit(0, 0, 0)); // TODO

    public final Color8Bit colorValue;

    GamePiece(Color8Bit color) {
      this.colorValue = color;
    }

    public boolean matches(Color8Bit otherColor) {
      double deltaRed = Math.abs(otherColor.red - this.colorValue.red);
      double deltaGreen = Math.abs(otherColor.green - this.colorValue.green);
      double deltaBlue = Math.abs(otherColor.blue - this.colorValue.blue);

      return deltaRed < Intake.kGamepeiceColorTolerance
          && deltaGreen < Intake.kGamepeiceColorTolerance
          && deltaBlue < Intake.kGamepeiceColorTolerance;
    }
  }
}
