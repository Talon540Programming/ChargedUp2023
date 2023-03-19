package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.constants.RobotDimensions;

public class ResetArmExtension extends CommandBase {
  public static final double kCurrentThresholdAmps = 10;

  private final ArmBase m_armBase;

  public ResetArmExtension(ArmBase armBase) {
    this.m_armBase = armBase;
    addRequirements(armBase);
  }

  @Override
  public void initialize() {
    m_armBase.setDisabled(true);
    m_armBase.setExtensionVoltage(-8.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_armBase.stopExtension();
    m_armBase.resetExtensionDistance(RobotDimensions.Arm.kFullyRetractedLengthMeters);
    m_armBase.setDisabled(false);
  }

  @Override
  public boolean isFinished() {
    return m_armBase.m_armExtensionInputs.CurrentAmps >= kCurrentThresholdAmps;
  }
}
