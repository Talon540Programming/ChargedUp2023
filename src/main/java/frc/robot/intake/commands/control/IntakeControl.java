package frc.robot.intake.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;

public class IntakeControl extends CommandBase {
  private final IntakeBase m_intakeBase;

  protected double kWristPercent, kClawPercent;

  public IntakeControl(IntakeBase intakeBase) {
    this.m_intakeBase = intakeBase;

    addRequirements(intakeBase);
  }

  @Override
  public void execute() {
    m_intakeBase.setClawVoltage(kClawPercent * 12.0);
    m_intakeBase.setWristVoltage(kWristPercent * 12.0);
  }
}
