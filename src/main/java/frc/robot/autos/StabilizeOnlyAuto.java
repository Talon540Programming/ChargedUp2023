package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

/**
 * Represents an Auto Routine used to drive forward at a given percent until the robot begins to incline or decline, then it will stabilize itself then end.
 */
public class StabilizeOnlyAuto extends SequentialCommandGroup {
    /**
     * Create a StabilizeOnlyAuto Auto.
     *
     * @param drivebase drivetrain subsystem.
     * @param speed the speed to drive at while seeking.
     */
    public StabilizeOnlyAuto(DriveBase drivebase, double speed) {
        super(
            Commands.deadline(
                    Commands.waitUntil(() -> !drivebase.isLevel()),
                    Commands.run(() -> drivebase.tankDrivePercent(
                            MathUtil.clamp(speed, -1, 1),
                            MathUtil.clamp(speed, -1, 1)))),
            new StabilizeRobot(drivebase)
        );
    }
}
