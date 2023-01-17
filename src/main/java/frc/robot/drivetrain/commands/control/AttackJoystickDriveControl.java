package frc.robot.drivetrain.commands.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.drivetrain.DrivetrainBase;

/** Command used to drive the drivetrain using a pair of AttackJoysticks. */
public class AttackJoystickDriveControl extends DrivetrainControl {
  private final Joystick leftJoystick, rightJoystick;

  /**
   * Construct a command that can be used to control the drivetrain using AttackJoysticks
   *
   * @param drivetrainBase Drivetrain subsystem.
   * @param leftJoystick left attack joystick.
   * @param rightJoystick right attack joystick.
   */
  public AttackJoystickDriveControl(
      DrivetrainBase drivetrainBase, Joystick leftJoystick, Joystick rightJoystick) {
    super(drivetrainBase);
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  /**
   * Construct a command that can be used to control the drivetrain using AttackJoysticks
   *
   * @param drivetrainBase Drivetrain subsystem.
   * @param leftJoystick left attack joystick.
   * @param rightJoystick right attack joystick.
   */
  public AttackJoystickDriveControl(
      DrivetrainBase drivetrainBase, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    this(drivetrainBase, leftJoystick.getHID(), rightJoystick.getHID());
  }

  @Override
  public void execute() {
    kLeftPercent = MathUtil.applyDeadband(leftJoystick.getY(), 0.1);
    kRightPercent = MathUtil.applyDeadband(rightJoystick.getY(), 0.1);
    super.execute();
  }
}
