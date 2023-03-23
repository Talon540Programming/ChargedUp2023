package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.lib.SparkMaxBurnManager;
import frc.lib.SparkMaxPeriodicFrameConfig;
import frc.robot.constants.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  public IntakeIOSparkMax(int leftId, int rightId, double conversionFactor) {
    m_leftMotor = new CANSparkMax(leftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightId, CANSparkMaxLowLevel.MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_leftMotor.restoreFactoryDefaults();
      m_rightMotor.restoreFactoryDefaults();
    }

    m_leftMotor.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);
    m_rightMotor.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);

    SparkMaxPeriodicFrameConfig.configureLeader(m_leftMotor);
    SparkMaxPeriodicFrameConfig.configureFollower(m_rightMotor);

    m_rightMotor.follow(m_leftMotor, true);

    m_leftMotor.setSmartCurrentLimit(20);
    m_rightMotor.setSmartCurrentLimit(20);

    m_leftMotor.enableVoltageCompensation(12.0);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setMeasurementPeriod(10);
    m_leftEncoder.setAverageDepth(2);
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_leftEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setMeasurementPeriod(10);
    m_rightEncoder.setAverageDepth(2);
    m_rightEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_leftMotor.setCANTimeout(0);
    m_rightMotor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_leftMotor.burnFlash();
      m_rightMotor.burnFlash();
    }

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.CurrentAmps =
        new double[] {m_leftMotor.getOutputCurrent(), m_rightMotor.getOutputCurrent()};
    inputs.TemperatureCelsius =
        new double[] {m_leftMotor.getMotorTemperature(), m_rightMotor.getMotorTemperature()};

    inputs.LeftVelocityRadPerSecond = m_leftEncoder.getVelocity();
    inputs.RightVelocityRadPerSecond = m_rightEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    m_leftMotor.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leftMotor.setIdleMode(mode.toIdleMode());
    m_rightMotor.setIdleMode(mode.toIdleMode());
  }
}
