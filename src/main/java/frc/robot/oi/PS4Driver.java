package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.commands.DriveControl;

public class PS4Driver implements DriverInterface {
  private final CommandPS4Controller m_controller;

  public PS4Driver(int port) {
    this.m_controller = new CommandPS4Controller(port);
  }

  @Override
  public double getLeftPercent(DriveControl.DriveMode mode) {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.15);
  }

  @Override
  public double getRightPercent(DriveControl.DriveMode mode) {
    return switch (mode) {
      case Arcade -> MathUtil.applyDeadband(-m_controller.getRightX(), 0.15);
      case Differential -> MathUtil.applyDeadband(-m_controller.getRightY(), 0.15);
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
