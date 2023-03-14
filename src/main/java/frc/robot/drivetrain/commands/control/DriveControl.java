package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class DriveControl extends CommandBase {
  public enum DriveMode {
    Differential,
    Arcade
  }

  protected double kLeftPercent, kRightPercent;
  private final DriveBase m_driveBase;

  private static final LoggedDashboardChooser<Double> m_speedLimiter =
      new LoggedDashboardChooser<>("Drive Speed Limiter");

  private static final LoggedDashboardChooser<DriveMode> m_driveMode =
      new LoggedDashboardChooser<>("Drive Mode");

  static {
    m_speedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_speedLimiter.addOption("Fast (70%)", 0.7);
    m_speedLimiter.addOption("Medium (30%)", 0.3);
    m_speedLimiter.addOption("Slow (15%)", 0.15);

    m_driveMode.addDefaultOption("Differential (Tank) Drive", DriveMode.Differential);
    m_driveMode.addOption("Arcade Drive", DriveMode.Arcade);
  }

  protected DriveControl(DriveBase driveBase) {
    this.m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    kLeftPercent *= m_speedLimiter.get();
    kRightPercent *= m_speedLimiter.get();

    switch (m_driveMode.get()) {
      case Differential -> this.m_driveBase.tankDrivePercent(kLeftPercent, kRightPercent);
      case Arcade -> this.m_driveBase.arcadeDrivePercent(kLeftPercent, kRightPercent);
    }
  }
}
