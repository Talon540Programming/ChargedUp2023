package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.oi.OperatorInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ArmControlVoltage extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  private final LoggedDashboardChooser<Double> m_rotationSpeedLimiter =
      new LoggedDashboardChooser<>("Arm Rotation Limiter");

  private final LoggedDashboardChooser<Double> m_extensionSpeedLimiter =
      new LoggedDashboardChooser<>("Arm Extension Limiter");

  public ArmControlVoltage(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    m_rotationSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_rotationSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_rotationSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_rotationSpeedLimiter.addOption("Slow (15%)", 0.15);

    m_extensionSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_extensionSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_extensionSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_extensionSpeedLimiter.addOption("Slow (15%)", 0.15);

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    m_armBase.setRotationVoltage(
        m_operatorInterface.getRotationLinearY() * 12.0 * m_rotationSpeedLimiter.get());
    m_armBase.setExtensionVoltage(
        m_operatorInterface.getExtensionPercent() * 12.0 * m_extensionSpeedLimiter.get());
  }
}
