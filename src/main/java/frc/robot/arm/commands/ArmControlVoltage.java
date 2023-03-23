package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.oi.OperatorInterface;

public class ArmControlVoltage extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  public ArmControlVoltage(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    m_armBase.setExtensionVoltage(m_operatorInterface.getExtensionPercent() * 12.0);
    m_armBase.setRotationVoltage(m_operatorInterface.getRotationLinearY() * 12.0);
  }
}
