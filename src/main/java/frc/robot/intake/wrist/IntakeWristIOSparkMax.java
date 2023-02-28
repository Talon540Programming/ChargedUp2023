package frc.robot.intake.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.Constants;

public class IntakeWristIOSparkMax implements IntakeWristIO {
  private final CANSparkMax m_wristMotor;
  private final RelativeEncoder m_wristEncoder;

  public IntakeWristIOSparkMax(int id) {
    m_wristMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);

    m_wristEncoder = m_wristMotor.getEncoder();
    m_wristEncoder.setMeasurementPeriod(10);
    m_wristEncoder.setAverageDepth(2);

    m_wristEncoder.setPositionConversionFactor(Constants.Intake.kWristPositionConversionFactor);
    m_wristEncoder.setVelocityConversionFactor(Constants.Intake.kWristVelocityConversionFactor);

    // TODO, configure Status Frame

    m_wristMotor.setCANTimeout(0);
  }

  @Override
  public void updateInputs(IntakeWristInputs inputs) {
    inputs.CurrentAmps = m_wristMotor.getOutputCurrent();
    inputs.TemperatureCelsius = m_wristMotor.getMotorTemperature();
    inputs.VelocityRadPerSecond = m_wristEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    m_wristMotor.setVoltage(voltage);
  }
}
