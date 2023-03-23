package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Operator implements OperatorInterface {
  private final CommandPS4Controller m_controller;

  public PS4Operator(int port) {
    this.m_controller = new CommandPS4Controller(port);
  }

  @Override
  public double getRotationLinearX() {
    return MathUtil.applyDeadband(m_controller.getLeftX(), .15);
  }

  @Override
  public double getRotationLinearY() {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), .15);
  }

  @Override
  public double getExtensionPercent() {
    return MathUtil.applyDeadband(-m_controller.getRightY(), .15);
  }

  @Override
  public Trigger lockRotation() {
    return m_controller.L1();
  }

  @Override
  public Trigger resetExtension() {
    return m_controller.share();
  }

  @Override
  public double getIntakePercent() {
    return m_controller.getR2Axis() - m_controller.getL2Axis();
  }
}
