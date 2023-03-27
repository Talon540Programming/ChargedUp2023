package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDriver implements DriverInterface {
  private final CommandXboxController m_controller;

  public XboxDriver(int port) {
    this.m_controller = new CommandXboxController(port);
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
    return m_controller.back();
  }

  @Override
  public Trigger enableCoastMode() {
    return m_controller.start();
  }

  @Override
  public Trigger toggleBalanceMode() {
    return m_controller.start();
  }
}
