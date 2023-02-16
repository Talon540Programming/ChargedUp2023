package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class RobotLimits {
    public static final double kMaxExtensionHorizontalInches = 48;
    public static final double kMaxExtensionHorizontalMeters = Units.inchesToMeters(kMaxExtensionHorizontalInches);

    public static final double kMaxExtensionVerticalInches = 6 * 12 + 6;
    public static final double kMaxExtensionVerticalMeters = Units.inchesToMeters(kMaxExtensionVerticalInches);

    public static final double kMinArmAngleDeg = -30; // TODO
    public static final double kMinArmAngleRad = Math.toRadians(kMinArmAngleDeg);
    public static final double kMaxArmAngleDeg = 210; // TODO
    public static final double kMaxArmAngleRad = Math.toRadians(kMaxArmAngleDeg);
}
