package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperator implements OperatorInterface {
  private final CommandXboxController m_controller;

  public XboxOperator(int port) {
    this.m_controller = new CommandXboxController(port);
  }

  @Override
  public double getRotationLinearX() {
    return MathUtil.applyDeadband(m_controller.getLeftX(), 0.15);
  }

  @Override
  public double getRotationLinearY() {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.15);
  }

  @Override
  public double getExtensionPercent() {
    return MathUtil.applyDeadband(-m_controller.getRightY(), 0.15);
  }

  @Override
  public Trigger lockRotation() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger resetExtension() {
    return m_controller.back();
  }

  @Override
  public double getIntakePercent() {
    return MathUtil.applyDeadband(m_controller.getRightTriggerAxis(), 0.15)
        - MathUtil.applyDeadband(m_controller.getLeftTriggerAxis(), 0.15);
  }
}
