package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.CalibrateArmExtension;
import frc.robot.arm.commands.GoToState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.FollowTrajectory;
import frc.robot.intake.IntakeBase;
import frc.robot.intake.commands.EjectIntake;
import java.util.List;

public class BottomCubeTwoCube extends SequentialCommandGroup {
  public BottomCubeTwoCube(DriveBase driveBase, ArmBase armBase, IntakeBase intakeBase) {
    List<PathPlannerTrajectory> trajectories =
        PathPlanner.loadPathGroup("BottomCubeTwoCube", RobotLimits.kTrajectoryConstraints);

    Pose2d initialPose = trajectories.get(0).getInitialPose();

    addCommands(
        Commands.runOnce(() -> driveBase.resetPosition(initialPose), driveBase),
        Commands.either(
            Commands.none(), new CalibrateArmExtension(armBase), armBase::extensionCalibrated),
        new GoToState(
            armBase,
            Constants.Arm.kArmKinematics.calculateArmState(
                initialPose,
                FieldConstants.kGrid[0][1].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)),
        new EjectIntake(intakeBase),
        Commands.runOnce(() -> intakeBase.setVoltage(8), intakeBase),
        new GoToState(armBase, ArmState.FLOOR),
        new FollowTrajectory(driveBase, trajectories.get(0)),
        new GoToState(
            armBase,
            Constants.Arm.kArmKinematics.calculateArmState(
                trajectories.get(1).getInitialPose(),
                FieldConstants.kGrid[1][1].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)),
        new EjectIntake(intakeBase),
        new FollowTrajectory(driveBase, trajectories.get(1)),
        new GoToState(armBase, ArmState.IDLE));
  }
}
