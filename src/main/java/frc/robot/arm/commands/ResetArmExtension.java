package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;

public class ResetArmExtension extends CommandBase {
  public static final double kCurrentThresholdAmps = 10; // TODO

  private final ArmBase m_armBase;

  public ResetArmExtension(ArmBase armBase) {
    this.m_armBase = armBase;
    addRequirements(armBase);
  }

  @Override
  public void initialize() {
    m_armBase.setDisabled(true);
    m_armBase.setExtensionVoltage(-4.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_armBase.stopExtension();

    if (!interrupted) m_armBase.completeExtensionCalibration();

    m_armBase.setDisabled(false);
  }

  @Override
  public boolean isFinished() {
    return m_armBase.m_armExtensionInputs.CurrentAmps >= kCurrentThresholdAmps;
  }
}
