package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;
import frc.robot.oi.OperatorInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class IntakeControl extends CommandBase {
  private final IntakeBase m_intakeBase;
  private final OperatorInterface m_operatorInterface;

  private final LoggedDashboardChooser<Double> m_speedLimiter =
      new LoggedDashboardChooser<>("Intake Speed Limiter");

  public IntakeControl(IntakeBase intakeBase, OperatorInterface operatorInterface) {
    this.m_intakeBase = intakeBase;
    this.m_operatorInterface = operatorInterface;

    m_speedLimiter.addDefaultOption("50% (Default)", 0.5);
    m_speedLimiter.addOption("30%", 0.3);
    m_speedLimiter.addOption("15%", 0.15);

    addRequirements(intakeBase);
  }

  @Override
  public void execute() {
    m_intakeBase.setVoltage(12.0 * m_operatorInterface.getIntakePercent() * m_speedLimiter.get());
  }
}
