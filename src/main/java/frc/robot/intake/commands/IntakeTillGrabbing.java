package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;

public class IntakeTillGrabbing extends CommandBase {
  private final IntakeBase m_intakeBase;

  private final Timer m_timer = new Timer();

  private final double m_timeoutSeconds;

  public IntakeTillGrabbing(IntakeBase intakeBase, double timeoutSeconds) {
    m_intakeBase = intakeBase;
    m_timeoutSeconds = timeoutSeconds;

    addRequirements(intakeBase);
  }

  public IntakeTillGrabbing(IntakeBase intakeBase) {
    this(intakeBase, Double.POSITIVE_INFINITY);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_intakeBase.setVoltage(8);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeBase.stop();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_intakeBase.isHoldingGamePiece() || m_timer.hasElapsed(m_timeoutSeconds);
  }
}
