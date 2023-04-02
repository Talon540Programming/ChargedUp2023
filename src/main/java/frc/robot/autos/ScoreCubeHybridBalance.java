package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.DriveTillCondition;
import frc.robot.drivetrain.commands.StabilizeRobot;

public class ScoreCubeHybridBalance extends SequentialCommandGroup {
  public ScoreCubeHybridBalance(DriveBase driveBase) {
    super(
        new DriveTime(driveBase, 0.75, -0.5),
        new DriveTillCondition(
            driveBase,
            () ->
                Math.abs(Math.toDegrees(driveBase.m_driveInputs.PitchPositionRad))
                    >= Constants.Drivetrain.kRobotStabilizationToleranceDegrees,
            0.1),
        new StabilizeRobot(driveBase));
  }
}
