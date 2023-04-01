package frc.robot.arm.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;

public class CalibrateArmExtension extends CommandBase {
  private final ArmBase m_armBase;

  private final Timer m_timer = new Timer();

  public CalibrateArmExtension(ArmBase armBase) {
    this.m_armBase = armBase;
    addRequirements(armBase);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_armBase.setDisabled(true);
  }

  @Override
  public void execute() {
    // This need to be called periodically to bypass the disabled voltage application
    m_armBase.setExtensionVoltage(-4.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_armBase.stopExtension();

    if (!interrupted) m_armBase.completeExtensionCalibration();

    m_armBase.setDisabled(false);
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_armBase.isExtensionStalled() && m_timer.hasElapsed(0.1);
  }
}
