package frc.robot.groups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.GoToState;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

public class AutoBalance extends SequentialCommandGroup {
  public AutoBalance(DriveBase driveBase, ArmBase armBase, boolean endWithArmUp) {
    addCommands(
        Commands.race(
            new StabilizeRobot(driveBase),
            Commands.run(
                () ->
                    armBase.updateState(
                        new ArmState(
                            Rotation2d.fromRadians(
                                (Math.PI / 2) + driveBase.m_driveInputs.PitchPositionRad),
                            RobotLimits.kMinArmLengthMeters)),
                armBase)),
        Commands.either(
            new GoToState(armBase, ArmState.IDLE), Commands.none(), () -> endWithArmUp));
  }
}
