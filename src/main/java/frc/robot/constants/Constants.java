package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.arm.ArmKinematics;

public final class Constants {
  /**
   * Whether advanced logging should be enabled. This can be disabled if there is too much going on.
   */
  public static final boolean kAdvancedLoggingEnabled = true;

  private static RobotType kRobotType = RobotType.ROBOT_2023C;
  public static final double loopPeriodSecs = 0.02;

  public enum RobotMode {
    REAL,
    REPLAY,
    SIM
  }

  public enum RobotType {
    ROBOT_2023C,
    ROBOT_SIMBOT
  }

  public static RobotType getRobotType() {
    if (RobotBase.isReal() && kRobotType == RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to SIM but it isn't a SIM, setting it to Competition Robot as redundancy.",
          false);
      kRobotType = RobotType.ROBOT_2023C;
    }

    if (RobotBase.isSimulation() && kRobotType != RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to REAL but it is a SIM, setting it to SIMBOT as redundancy.", false);
      kRobotType = RobotType.ROBOT_SIMBOT;
    }

    return kRobotType;
  }

  public static RobotMode getRobotMode() {
    return switch (getRobotType()) {
      case ROBOT_2023C -> RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
      case ROBOT_SIMBOT -> RobotMode.SIM;
    };
  }

  public static final class Drivetrain {
    public static final boolean kLeftSideInverted = false;
    public static final boolean kLeftSensorInverted = false;
    public static final boolean kRightSideInverted = true;
    public static final boolean kRightSensorInverted = true;

    public static final double kDrivetrainGearRatio = (54.0 / 20.0) * (50.0 / 11.0);

    public static final double kTrackWidthInches = 19.618320; // TODO test in SysID
    public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

    public static final double kWheelRadiusInches = 3;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

    public static final DifferentialDriveKinematics kDrivetrainKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final NeutralMode kDrivetrainDefaultNeutralMode = NeutralMode.COAST;

    public static class ControlValues {
      public static class Characterization {
        public static final double kSLinear = 0.22; // TODO
        public static final double kVLinear = 1.98; // TODO this is a sim value, real value must be found in sysid
        public static final double kALinear = 0.2; // TODO this is a sim value, real value must be found in sysid

        public static final double kVAngular = 1.5; // TODO this is a sim value, real value must be found in sysid
        public static final double kAAngular = 0.3; // TODO this is a sim value, real value must be found in sysid
      }

      public static class Stabilization {
        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }

      public static class Trajectory {
        public static final double kP = 8.5; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
      }
    }
  }

  public static final class Arm {
    public static final ArmKinematics kArmKinematics =
        new ArmKinematics(RobotDimensions.Arm.kFulcrumPose);

    public static final boolean kRotationInverted = false;
    public static final boolean kExtensionInverted = false;

    @SuppressWarnings("PointlessArithmeticExpression")
    public static final double kRotationGearRatio =
        (4.0 / 1.0) * (10.0 / 1.0) * (66.0 / 18.0); // TODO

    @SuppressWarnings("PointlessArithmeticExpression")
    public static final double kExtensionGearRatio = (10.0 / 1.0) * (4.0 / 1.0);

    public static final double kExtensionWinchRadiusInches = (3.0 / 4.0) / 2.0;
    public static final double kExtensionWinchRadiusMeters =
        Units.inchesToMeters(kExtensionWinchRadiusInches);

    public static final double kExtensionConversionFactor =
        (1 / kExtensionGearRatio) * 2 * Math.PI * kExtensionWinchRadiusMeters;

    public static final double kRotationAbsoluteEncoderOffsetDegrees = -37.596;

    public static class ControlValues {
      public static class RotationValues {
        public static final double kP = 5; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 1; // TODO
      }

      public static class ExtensionValues {
        public static final double kP = 4; // TODO this is a sim value and should not be used
        public static final double kI = 0; // TODO this is a sim value and should not be used
        public static final double kD = 0; // TODO this is a sim value and should not be used
      }
    }
  }

  public static final class Intake {
    public static final double kGearRatio = 4.0;
    public static final double kConversionFactor = 1 / kGearRatio * 2.0 * Math.PI;
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
}
