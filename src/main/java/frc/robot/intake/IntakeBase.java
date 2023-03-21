package frc.robot.intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import java.util.Optional;
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

  public Optional<Constants.GamePiece> getSeenGamePiece() {
    Color8Bit currentColor = m_intakeIO.getColor8Bit();
    for (Constants.GamePiece piece : Constants.GamePiece.values()) {
      if (piece.matches(currentColor)) return Optional.of(piece);
    }

    return Optional.empty();
  }
}
