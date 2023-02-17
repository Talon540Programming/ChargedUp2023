package frc.robot.arm.command.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class ArmControl extends CommandBase {
  private final ArmBase m_armBase;
  protected double kRotationPercent, kExtensionPercent;

  private static final LoggedDashboardChooser<Double> m_rotationSpeedLimiter =
      new LoggedDashboardChooser<>("Rotation Speed Limit");
  private static final LoggedDashboardChooser<Double> m_extensionSpeedLimiter =
      new LoggedDashboardChooser<>("Extension Speed Limit");

  static {
    m_rotationSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_rotationSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_rotationSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_rotationSpeedLimiter.addOption("Slow (15%)", 0.15);

    m_extensionSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_extensionSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_extensionSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_extensionSpeedLimiter.addOption("Slow (15%)", 0.15);
  }

  public ArmControl(ArmBase armBase) {
    this.m_armBase = armBase;

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    kRotationPercent *= m_rotationSpeedLimiter.get();
    kExtensionPercent *= m_extensionSpeedLimiter.get();

    m_armBase.setRotationVoltage(kRotationPercent * 12.0);
    m_armBase.setExtensionVoltage(kExtensionPercent * 12.0);
  }
}
