package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PS4DriverOperator implements DriverInterface, OperatorInterface {
  private final CommandPS4Controller m_controller;

  private final LoggedDashboardChooser<DriveMode> m_driveMode =
      new LoggedDashboardChooser<>("Drive Mode");

  private final LoggedDashboardChooser<Double> m_speedLimiter =
      new LoggedDashboardChooser<>("Drive Speed Limiter");

  public PS4DriverOperator(int port) {
    this.m_controller = new CommandPS4Controller(port);

    m_driveMode.addDefaultOption("Arcade Drive", DriveMode.Arcade);
    m_driveMode.addOption("Differential (Tank) Drive", DriveMode.Differential);

    m_speedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_speedLimiter.addOption("Fast (70%)", 0.7);
    m_speedLimiter.addOption("Medium (30%)", 0.3);
    m_speedLimiter.addOption("Slow (15%)", 0.15);
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
    return m_controller.touchpad();
  }

  @Override
  public double getArmRotationPercent() {
    return m_controller.povUp().getAsBoolean()
        ? 0.5
        : m_controller.povDown().getAsBoolean() ? -0.5 : 0;
  }

  @Override
  public double getArmExtensionPercent() {
    return m_controller.povRight().getAsBoolean()
        ? 0.5
        : m_controller.povLeft().getAsBoolean() ? -0.5 : 0;
  }
}
