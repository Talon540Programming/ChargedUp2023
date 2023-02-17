package frc.robot.arm.extension;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface ArmExtensionIO extends LoggedIO<ArmExtensionIO.ArmExtensionIOInputs> {
  @AutoLog
  class ArmExtensionIOInputs {
    public double DistanceTraveledMeters;
    public double VelocityRadiansPerSecond;
  }

  @Override
  default void updateInputs(ArmExtensionIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setDistance(double distance) {}

  default void resetDistance() {}

  default void setNeutralMode(Constants.NeutralMode mode) {}
}
