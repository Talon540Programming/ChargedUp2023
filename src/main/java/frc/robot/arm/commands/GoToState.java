package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;

public class GoToState extends CommandBase {
  private final ArmBase m_armBase;
  private final ArmState m_setpoint;

  public GoToState(ArmBase armBase, ArmState setpoint) {
    this.m_armBase = armBase;
    this.m_setpoint = setpoint;

    addRequirements(armBase);
  }

  @Override
  public void initialize() {
    m_armBase.updateState(m_setpoint);
  }

  @Override
  public boolean isFinished() {
    return m_armBase.atSetpoint();
  }
}
