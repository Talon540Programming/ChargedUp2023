package frc.robot.intake.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.constants.Constants;

public class IntakeClawIOSparkMax implements IntakeClawIO {
  private final CANSparkMax m_clawMotor;

  public IntakeClawIOSparkMax(int id) {
    m_clawMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_clawMotor.enableVoltageCompensation(12);
    m_clawMotor.setSmartCurrentLimit(20);

    setNeutralMode(Constants.NeutralMode.BRAKE);

    // TODO, configure Status Frame

    m_clawMotor.setCANTimeout(0);
  }

  @Override
  public void updateInputs(IntakeClawIO.IntakeClawInputs inputs) {
    inputs.CurrentAmps = m_clawMotor.getOutputCurrent();
    inputs.TemperatureCelsius = m_clawMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    m_clawMotor.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_clawMotor.setIdleMode(mode.toIdleMode());
  }
}
