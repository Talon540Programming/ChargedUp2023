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
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.15);
  }

  @Override
  public double getRightPercent(DriveMode mode) {
    return switch (mode) {
      case Arcade -> MathUtil.applyDeadband(-m_controller.getRightX(), 0.15);
      case Differential -> MathUtil.applyDeadband(-m_controller.getRightY(), 0.15);
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
