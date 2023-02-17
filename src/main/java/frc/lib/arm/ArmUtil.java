package frc.lib.arm;

import frc.robot.constants.RobotDimensions;

public class ArmUtil {
  public static double calculateArmLength(double lengthExtendedMeters) {
    return RobotDimensions.Arm.kFullyRetractedLengthMeters + lengthExtendedMeters; // TODO
  }

  public static double calculateExtensionLength(double armLength) {
    return 0; // TODO
  }

  public static double calculateCenterOfMassOffset(double armLength) {
    return 0; // TODO
  }
}
