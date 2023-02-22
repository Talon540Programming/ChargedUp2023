package frc.robot.arm.extension;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.Constants;

/**
 * ArmExtensionIO using 1 SparkMax motor controller.
 */
public class ArmExtensionIONeo implements ArmExtensionIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public ArmExtensionIONeo(int id, boolean motorInverted, boolean encoderInverted) {
    motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

    setNeutralMode(Constants.NeutralMode.BRAKE);

    motor.setInverted(motorInverted);

    encoder = motor.getEncoder();
    encoder.setInverted(encoderInverted);

    encoder.setPositionConversionFactor(Constants.Arm.kExtensionPositionConversionFactor);
    encoder.setVelocityConversionFactor(Constants.Arm.kExtensionVelocityConversionFactor);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    inputs.DistanceTraveledMeters = encoder.getPosition();
    inputs.VelocityRadiansPerSecond = encoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setDistance(double distanceMeters) {
    encoder.setPosition(distanceMeters);
  }

  @Override
  public void resetDistance() {
    encoder.setPosition(0);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    switch (mode) {
      case BRAKE -> motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      case COAST -> motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }
}
