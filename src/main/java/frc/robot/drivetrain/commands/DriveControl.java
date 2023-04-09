package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;
import frc.robot.oi.DriverInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveControl extends CommandBase {
  private final DriveBase m_driveBase;
  private final DriverInterface m_driverInterface;

  private final LoggedDashboardChooser<Double> m_leftSpeedLimiter =
      new LoggedDashboardChooser<>("Left Drive Speed Limiter");

  private final LoggedDashboardChooser<Double> m_rightSpeedLimiter =
      new LoggedDashboardChooser<>("Right Drive Speed Limiter");

  private final LoggedDashboardChooser<DriveMode> m_driveMode =
      new LoggedDashboardChooser<>("Drive Mode");

  public DriveControl(DriveBase driveBase, DriverInterface driverInterface) {
    this.m_driveBase = driveBase;
    this.m_driverInterface = driverInterface;

    m_leftSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_leftSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_leftSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_leftSpeedLimiter.addOption("Slow (15%)", 0.15);

    m_rightSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_rightSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_rightSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_rightSpeedLimiter.addOption("Slow (15%)", 0.15);

    m_driveMode.addDefaultOption("Arcade Drive", DriveMode.Arcade);
    m_driveMode.addOption("Differential (Tank) Drive", DriveMode.Differential);

    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    DriveMode mode = m_driveMode.get();

    switch (mode) {
      case Arcade -> m_driveBase.arcadeDrivePercent(
          m_driverInterface.getLeftPercent(mode) * m_leftSpeedLimiter.get(),
          m_driverInterface.getRightPercent(mode) * m_rightSpeedLimiter.get());
      case Differential -> m_driveBase.tankDrivePercent(
          m_driverInterface.getLeftPercent(mode) * m_leftSpeedLimiter.get(),
          m_driverInterface.getRightPercent(mode) * m_rightSpeedLimiter.get());
    }
  }

  public enum DriveMode {
    Differential,
    Arcade
  }
}
