package frc.robot.drivetrain.commands.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.drivetrain.DriveBase;

/** Command used to drive the drivetrain using a pair of AttackJoysticks. */
public class AttackJoystickDriveControl extends DrivetrainControl {
  private final Joystick leftJoystick, rightJoystick;

  /**
   * Construct a command that can be used to control the drivetrain using AttackJoysticks
   *
   * @param driveBase Drivetrain subsystem.
   * @param leftJoystick left attack joystick.
   * @param rightJoystick right attack joystick.
   */
  public AttackJoystickDriveControl(
          DriveBase driveBase, Joystick leftJoystick, Joystick rightJoystick) {
    super(driveBase);
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  /**
   * Construct a command that can be used to control the drivetrain using AttackJoysticks
   *
   * @param driveBase Drivetrain subsystem.
   * @param leftJoystick left attack joystick.
   * @param rightJoystick right attack joystick.
   */
  public AttackJoystickDriveControl(
          DriveBase driveBase, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    this(driveBase, leftJoystick.getHID(), rightJoystick.getHID());
  }

  @Override
  public void execute() {
    kLeftPercent = MathUtil.applyDeadband(leftJoystick.getY(), 0.1);
    kRightPercent = MathUtil.applyDeadband(rightJoystick.getY(), 0.1);
    super.execute();
  }
}
