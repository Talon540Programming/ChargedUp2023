package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class RobotLimits {
  public static final double kMaxExtensionHorizontalInches = 48; // 48" Extension Limit
  public static final double kMaxExtensionHorizontalMeters =
      Units.inchesToMeters(kMaxExtensionHorizontalInches);

  public static final double kMaxExtensionVerticalInches = 78; // 6'6" Max Height Limit
  public static final double kMaxExtensionVerticalMeters =
      Units.inchesToMeters(kMaxExtensionVerticalInches);

  public static final double kMaxArmAngleRadians = Math.PI; // TODO
  public static final double kMinArmAngleRadians = 0; // TODO

  public static final double kMaxDrivetrainVelocityMetersPerSecond =
      4; // TODO, determine the best value
  public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared =
      3.5; // TODO, determine the best value

  public static final double kMaxArmVelocityRadPerSecond = 0; // TODO
  public static final double kMaxArmAccelerationRadPerSecondSquared = 0; // TODO
}
