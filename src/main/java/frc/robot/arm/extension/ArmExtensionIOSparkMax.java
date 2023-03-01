package frc.robot.arm.extension;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.Constants;

/** ArmExtensionIO using 1 SparkMax motor controller. */
public class ArmExtensionIOSparkMax implements ArmExtensionIO {
  private final CANSparkMax m_winchMotor;
  private final RelativeEncoder m_winchEncoder;

  public ArmExtensionIOSparkMax(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double positionConversionFactor,
      double velocityConversionFactor) {
    m_winchMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_winchMotor.setInverted(motorInverted);

    m_winchMotor.setSmartCurrentLimit(40);

    m_winchMotor.enableVoltageCompensation(12.0);

    m_winchEncoder = m_winchMotor.getEncoder();
    m_winchEncoder.setInverted(encoderInverted);
    m_winchEncoder.setMeasurementPeriod(10);
    m_winchEncoder.setAverageDepth(2);
    resetDistance();

    m_winchEncoder.setPositionConversionFactor(positionConversionFactor);
    m_winchEncoder.setVelocityConversionFactor(velocityConversionFactor);

    m_winchMotor.setCANTimeout(0);

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    inputs.DistanceTraveledMeters = m_winchEncoder.getPosition();
    inputs.VelocityRadiansPerSecond = m_winchEncoder.getVelocity();
    inputs.CurrentAmps = m_winchMotor.getOutputCurrent();
    inputs.TemperatureCelsius = m_winchMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    m_winchMotor.setVoltage(voltage);
  }

  @Override
  public void setDistance(double distanceMeters) {
    m_winchEncoder.setPosition(distanceMeters);
  }

  @Override
  public void resetDistance() {
    m_winchEncoder.setPosition(0);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_winchMotor.setIdleMode(mode.toIdleMode());
  }
}
