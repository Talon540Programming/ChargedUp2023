package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.oi.OperatorInterface;

public class ArmControl extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  public ArmControl(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    m_armBase.setRotationPercent(m_operatorInterface.getArmRotationPercent());
    m_armBase.setExtensionPercent(m_operatorInterface.getArmExtensionPercent());
  }
}
