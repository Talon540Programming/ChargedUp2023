package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

/**
 * Represents an Auto Routine used to drive forward at a given percent until the robot begins to
 * incline or decline, then it will stabilize itself then end.
 */
public class StabilizeOnly extends SequentialCommandGroup {
  /**
   * Create a StabilizeOnly Auto.
   *
   * @param speed the speed to drive at while seeking.
   * @param drivebase drivetrain subsystem.
   */
  public StabilizeOnly(double speed, DriveBase drivebase) {
    super(
        Commands.deadline(
            Commands.waitUntil(() -> !drivebase.isLevel()),
            Commands.run(
                () ->
                    drivebase.tankDrivePercent(
                        MathUtil.clamp(speed, -1, 1), MathUtil.clamp(speed, -1, 1)))),
        new StabilizeRobot(drivebase));
  }
}
