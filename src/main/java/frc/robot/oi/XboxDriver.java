package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class XboxDriver implements DriverInterface {
  private final CommandXboxController m_controller;

  private final LoggedDashboardChooser<Double> m_speedLimiter =
      new LoggedDashboardChooser<>("Drive Speed Limiter");

  private final LoggedDashboardChooser<DriveMode> m_driveMode =
      new LoggedDashboardChooser<>("Drive Mode");

  public XboxDriver(int port) {
    this.m_controller = new CommandXboxController(port);

    m_speedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_speedLimiter.addOption("Fast (70%)", 0.7);
    m_speedLimiter.addOption("Medium (30%)", 0.3);
    m_speedLimiter.addOption("Slow (15%)", 0.15);

    m_driveMode.addDefaultOption("Arcade Drive", DriveMode.Arcade);
    m_driveMode.addOption("Differential (Tank) Drive", DriveMode.Differential);
  }

  @Override
  public DriveMode getDriveMode() {
    return m_driveMode.get();
  }

  @Override
  public double getLeftPercent() {
    return MathUtil.applyDeadband(m_controller.getLeftY(), 0.05) * m_speedLimiter.get();
  }

  @Override
  public double getRightPercent() {
    return MathUtil.applyDeadband(
            getDriveMode() == DriveMode.Differential
                ? m_controller.getRightY()
                : m_controller.getRightX(),
            0.05)
        * m_speedLimiter.get();
  }

  @Override
  public Trigger toggleBalanceMode() {
    return m_controller.start();
  }
}
