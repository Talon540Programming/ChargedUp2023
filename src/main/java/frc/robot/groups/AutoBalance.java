package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.GoToState;
import frc.robot.arm.commands.GoToSuppliedState;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

public class AutoBalance extends ParallelCommandGroup {
    public AutoBalance(DriveBase driveBase, ArmBase armBase) {
        super(
                new ParallelRaceGroup(
                        new StabilizeRobot(driveBase),
                        new GoToSuppliedState(armBase, () -> new ArmState(
                                (Math.PI / 2) + driveBase.m_driveInputs.PitchPositionRad,
                                RobotLimits.kMinArmLengthMeters
                        ))
                ),
                new GoToState(armBase, ArmState.IDLE)
        );
    }
}
