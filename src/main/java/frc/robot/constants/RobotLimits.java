package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class RobotLimits {
  public static final double kMaxExtensionHorizontalInches = 48; // 48" Extension Limit
  public static final double kMaxExtensionHorizontalMeters =
      Units.inchesToMeters(kMaxExtensionHorizontalInches);

  public static final double kMaxExtensionVerticalInches = 78; // 6'6" Max Height Limit
  public static final double kMaxExtensionVerticalMeters =
      Units.inchesToMeters(kMaxExtensionVerticalInches);

  public static final double kForwardLimitRadians = (11.0 * Math.PI) / 6.0;
  public static final double kRearLimit = (7 * Math.PI) / 6.0;
}
