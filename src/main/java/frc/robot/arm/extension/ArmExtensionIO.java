package frc.robot.arm.extension;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

/** IO abstraction used to control the extension of a telescoping arm. */
public interface ArmExtensionIO extends LoggedIO<ArmExtensionIO.ArmExtensionIOInputs> {
  @AutoLog
  class ArmExtensionIOInputs {
    public double DistanceTraveledMeters;
    public double VelocityMetersPerSecond;
    public double TemperatureCelsius;
    public double CurrentAmps;
  }

  @Override
  default void updateInputs(ArmExtensionIOInputs inputs) {}

  /**
   * Set the output voltage of the extension winch motor.
   *
   * @param voltage voltage to set.
   */
  default void setVoltage(double voltage) {}

  /**
   * Set the distance traveled by the winch,
   *
   * @param distanceMeters distance traveled by the winch in meters.
   */
  default void setDistance(double distanceMeters) {}

  /** Reset the distance traveled by the winch to zero. */
  default void resetDistance() {}

  /**
   * Set the neutral mode of the winch motor.
   *
   * @param mode NeutralMode to set.
   */
  default void setNeutralMode(Constants.NeutralMode mode) {}
}
