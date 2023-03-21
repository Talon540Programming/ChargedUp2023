package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;
import frc.robot.oi.OperatorInterface;

public class IntakeControl extends CommandBase {
  private final IntakeBase m_intakeBase;
  private final OperatorInterface m_operatorInterface;

  public IntakeControl(IntakeBase intakeBase, OperatorInterface operatorInterface) {
    this.m_intakeBase = intakeBase;
    this.m_operatorInterface = operatorInterface;

    addRequirements(intakeBase);
  }

  @Override
  public void execute() {
    m_intakeBase.setVoltage(12.0 * m_operatorInterface.getIntakePercent());
  }
}
