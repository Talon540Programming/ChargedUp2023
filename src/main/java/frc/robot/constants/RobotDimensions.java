package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// In general, we use metric units within the robot code. When importing from code,
// we use imperial units which is why we must convert it.
public class RobotDimensions {
  public static final double kRobotMassLbs = 128.25; // TODO
  public static final double kRobotMassKilos = Units.lbsToKilograms(kRobotMassLbs);

  public static final double kArmAndEffectorWeightLbs = Arm.kArmMassLbs + Effector.kEffectorMassLbs;
  public static final double kArmAndEffectorWeightKg =
      Units.lbsToKilograms(kArmAndEffectorWeightLbs);

  public static final class Drivetrain {
    public static final double kBumperWidthInches = 3.5;
    public static final double kBumperWidthMeters = Units.inchesToMeters(kBumperWidthInches);

    public static final double kDrivetrainWidthInches = 24;
    public static final double kDrivetrainWidthMeters =
        Units.inchesToMeters(kDrivetrainWidthInches);

    public static final double kDrivetrainLengthInches = 28;
    public static final double kDrivetrainLengthMeters =
        Units.inchesToMeters(kDrivetrainLengthInches);

    public static final double kDrivetrainWidthBumpersInches = 24 + kBumperWidthInches;
    public static final double kDrivetrainWidthBumpersMeters =
        Units.inchesToMeters(kDrivetrainWidthBumpersInches);

    public static final double kDrivetrainLengthBumpersInches = 28 + kBumperWidthInches;
    public static final double kDrivetrainLengthBumpersMeters =
        Units.inchesToMeters(kDrivetrainLengthBumpersInches);
  }

  public static final class Arm {
    public static final double kFulcrumHeightInches = 17;
    public static final double kFulcrumHeightMeters = Units.inchesToMeters(kFulcrumHeightInches);

    public static final Translation3d kFulcrumPose = new Translation3d(0, 0, kFulcrumHeightMeters);

    // First extrusion is from fulcrum, not end of extrusion
    public static final double kFirstExtrusionLengthInches = 17;
    public static final double kFirstExtrusionLengthMeters =
        Units.inchesToMeters(kFirstExtrusionLengthInches);

    // Second and Third extrusion lengths are without the retention hardware,
    // which are retained within their encapsulating extrusions.
    public static final double kSecondExtrusionLengthInches = 25.25;
    public static final double kSecondExtrusionLengthMeters =
        Units.inchesToMeters(kSecondExtrusionLengthInches);

    public static final double kThirdExtrusionLengthInches = 17;
    public static final double kThirdExtrusionLengthMeters =
        Units.inchesToMeters(kThirdExtrusionLengthInches);

    public static final double kFullyRetractedLengthInches = 20.625;
    public static final double kFullyRetractedLengthMeters =
        Units.inchesToMeters(kFullyRetractedLengthInches);

    public static final double kFullyExtendedLengthInches = 60;
    public static final double kFullyExtendedLengthMeters =
        Units.inchesToMeters(kFullyExtendedLengthInches);

    public static final double kArmMassLbs = 5.169;
    public static final double kArmMassKg = Units.lbsToKilograms(kArmMassLbs);
  }

  public static final class Effector {
    // This includes the arm mount's length
    public static final double kLengthInches = 15.250; // TODO
    public static final double kLengthMeters = Units.inchesToMeters(kLengthInches);

    public static final double kEffectorMassLbs = 6.721;
    public static final double kEffectorMassKg = Units.lbsToKilograms(kEffectorMassLbs);
  }
}
