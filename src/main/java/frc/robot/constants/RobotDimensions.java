package frc.robot.constants;

import edu.wpi.first.math.util.Units;

// In general, we use metric units within the robot code. When importing from code,
// we use imperial units which is why we must convert it.
public class RobotDimensions {
  public static final class Drivetrain {
    public static final double kBumperWidthInches = 3;
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

    public static final double kFullyRetractedLengthInches = 20.253; // TODO
    public static final double kFullyRetractedLengthMeters =
        Units.inchesToMeters(kFullyRetractedLengthInches); // TODO
  }

  public static final class Grabber {
    // This includes the mounting hardware onto the third extrusion which is why it lacks it.
    public static final double kLengthInches = 17; // TODO
    public static final double kLengthMeters = Units.inchesToMeters(kLengthInches);
  }
}