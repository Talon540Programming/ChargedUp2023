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
    return m_controller.L1();
  }

  @Override
  public Trigger calibrateExtension() {
    return m_controller.share();
  }

  @Override
  public double getIntakePercent() {
    return MathUtil.applyDeadband(m_controller.getR2Axis(), 0.15)
        - MathUtil.applyDeadband(m_controller.getL2Axis(), 0.15);
  }

  @Override
  public Trigger ejectIntake() {
    return m_controller.R1();
  }

  @Override
  public Trigger idle() {
    return m_controller.options();
  }

  @Override
  public Trigger singleSubstation() {
    return m_controller.povUp();
  }

  @Override
  public Trigger cubeHigh() {
    return m_controller.triangle();
  }

  @Override
  public Trigger cubeMid() {
    return m_controller.square();
  }

  @Override
  public Trigger coneMid() {
    return m_controller.circle();
  }

  @Override
  public Trigger hybrid() {
    return m_controller.cross();
  }
}
