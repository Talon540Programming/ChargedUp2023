package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotLimits {
  public static final double kMaxExtensionHorizontalInches = 48; // 48" Extension Limit
  public static final double kMaxExtensionHorizontalMeters =
      Units.inchesToMeters(kMaxExtensionHorizontalInches);

  public static final double kMaxExtensionVerticalInches = 78; // 6'6" Max Height Limit
  public static final double kMaxExtensionVerticalMeters =
      Units.inchesToMeters(kMaxExtensionVerticalInches);

  public static final double kMaxDrivetrainVelocityMetersPerSecond = 4; // TODO
  public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared = 3.5; // TODO

  public static final double kMaxArmVelocityRadPerSecond = Math.PI / 4;
  public static final double kMaxArmAccelerationRadPerSecondSquared = Math.PI / 2;

  public static final TrapezoidProfile.Constraints kArmRotationConstraints =
      new TrapezoidProfile.Constraints(
          kMaxArmVelocityRadPerSecond, kMaxArmAccelerationRadPerSecondSquared);

  public static final double kMinArmLengthInches = RobotDimensions.Arm.kFullyRetractedLengthInches;
  public static final double kMinArmLengthMeters = Units.inchesToMeters(kMinArmLengthInches);

  public static final double kMaxArmLengthInches = RobotDimensions.Arm.kFullyExtendedLengthInches;
  public static final double kMaxArmLengthMeters = Units.inchesToMeters(kMaxArmLengthInches);
}
