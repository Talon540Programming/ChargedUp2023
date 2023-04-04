package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.GoToSuppliedState;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

public class AutoBalance extends ParallelRaceGroup {
    public AutoBalance(DriveBase driveBase, ArmBase armBase) {
        super(
                new StabilizeRobot(driveBase),
                new GoToSuppliedState(armBase, () -> new ArmState(
                        (Math.PI / 2) - driveBase.m_driveInputs.PitchPositionRad,
                        RobotLimits.kMinArmLengthMeters
                ))
        );
    }
}
