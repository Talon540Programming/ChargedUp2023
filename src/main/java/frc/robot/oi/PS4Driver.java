package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Driver implements DriverInterface {
  private final CommandPS4Controller m_controller;

  public PS4Driver(int port) {
    this.m_controller = new CommandPS4Controller(port);
  }

  @Override
  public double getLeftPercent(DriveMode mode) {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.05);
  }

  @Override
  public double getRightPercent(DriveMode mode) {
    return switch (mode) {
      case Arcade -> m_controller.getRightX();
      case Differential -> -m_controller.getRightY();
    };
  }

  @Override
  public Trigger enableBrakeMode() {
    return m_controller.share();
  }

  @Override
  public Trigger enableCoastMode() {
    return m_controller.options();
  }

  @Override
  public Trigger toggleBalanceMode() {
    return m_controller.touchpad();
  }
}
