package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase {
  private final IntakeIO m_intakeIO;
  public final IntakeInputsAutoLogged m_intakeInputs = new IntakeInputsAutoLogged();

  public IntakeBase(IntakeIO intakeIO) {
    this.m_intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    m_intakeIO.updateInputs(m_intakeInputs);
    Logger.getInstance().processInputs("Intake", m_intakeInputs);
  }

  public void setVoltage(double voltage) {
    m_intakeIO.setVoltage(voltage);
  }

  public void stop() {
    m_intakeIO.setVoltage(0.0);
  }

  public boolean isHoldingGamePiece() {
    return m_intakeIO.isStalled();
  }
}
