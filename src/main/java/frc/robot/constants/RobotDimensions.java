package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

// In general, we use metric units within the robot code. When importing from code,
// we use imperial units which is why we must convert it.
public class RobotDimensions {
  public static final double kRobotMassPounds = 80; // TODO
  public static final double kRobotMassKilos = Units.lbsToKilograms(kRobotMassPounds);

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

    public static final Pose3d kFulcrumPose = new Pose3d(0, 0, kFulcrumHeightMeters, new Rotation3d());

    // First extrusion is from fulcrum, not end of extrusion
    public static final double kFirstExtrusionLengthInches = 17; // TODO
    public static final double kFirstExtrusionLengthMeters =
        Units.inchesToMeters(kFirstExtrusionLengthInches);

    // Second and Third extrusion lengths are without the end caps
    // which are retained within their encapsulating extrusions.
    public static final double kSecondExtrusionLengthInches = 25; // TODO
    public static final double kSecondExtrusionLengthMeters =
        Units.inchesToMeters(kSecondExtrusionLengthInches);

    public static final double kThirdExtrusionLengthInches = 18; // TODO
    public static final double kThirdExtrusionLengthMeters =
        Units.inchesToMeters(kThirdExtrusionLengthInches);

    public static final double kFullyRetractedLengthInches = 27.836956; // TODO
    public static final double kFullyRetractedLengthMeters =
        Units.inchesToMeters(kFullyRetractedLengthInches); // TODO

    public static final double kArmMassLbs = 6.097394715877;
    public static final double kArmMassKg = Units.lbsToKilograms(kArmMassLbs);
  }

  public static final class Effector {
    // This includes the mounting hardware onto the third extrusion which is why it lacks it.
    public static final double kLengthInches = 14.366; // TODO
    public static final double kLengthMeters = Units.inchesToMeters(kLengthInches);

    public static final double kEffectorMassLbs = 6.605233946065;
    public static final double kEffectorMassKg = Units.lbsToKilograms(kEffectorMassLbs);
  }
}
