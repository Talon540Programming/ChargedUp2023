package frc.lib.arm;

/** Utility class used to calculate values for the Arm. */
public class ArmUtil {
  /**
   * Calculate the total length of the Arm in meters based on how far the winch is extended in
   * meters.
   *
   * @param lengthExtendedMeters how far the winch is extended in meters.
   * @return estimated total length of the arm in meters.
   */
  public static double calculateArmLength(double lengthExtendedMeters) {
    return 0; // TODO
  }

  /**
   * Calculate how far the winch has extended based on the total length of the arm in meters.
   *
   * @param armLength total length of the arm in meters.
   * @return estimated length the winch extension.
   */
  public static double calculateExtensionLength(double armLength) {
    return 0; // TODO
  }

  /**
   * Calculate the delta distance of the center of mass of the arm and the fulcrum based on the
   * total length of the arm.
   *
   * @param armLength total length of the arm in meters.
   * @return estimated delta distance of the center of mass.
   */
  public static double calculateCenterOfMassOffset(double armLength) {
    return 0; // TODO
  }
}
