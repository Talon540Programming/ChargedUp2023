package frc.robot.arm.rotation;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

/** IO abstraction used to control an orm rotation mechanism. */
public interface ArmRotationIO extends LoggedIO<ArmRotationIO.ArmRotationIOInputs> {
  @AutoLog
  class ArmRotationIOInputs {
    public double[] CurrentAmps = new double[] {};
    public double[] TemperatureCelsius = new double[] {};
    public double ArmVelocityRadPerSecond;
    public double AbsoluteArmPositionRad;
  }

  @Override
  default void updateInputs(ArmRotationIOInputs inputs) {}

  /**
   * Set the output voltage of the rotation motor.
   *
   * @param voltage voltage to set.
   */
  default void setVoltage(double voltage) {}

  /**
   * Set the neutral mode of the rotation motors.
   *
   * @param mode NeutralMode to set.
   */
  default void setNeutralMode(Constants.NeutralMode mode) {}
}
