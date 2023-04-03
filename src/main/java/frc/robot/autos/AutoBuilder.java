package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.GeomUtil;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.CalibrateArmExtension;
import frc.robot.arm.commands.GoToState;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.FollowTrajectory;
import frc.robot.intake.IntakeBase;
import frc.robot.intake.commands.EjectIntake;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutoBuilder {
    private final DriveBase m_driveBase;
    private final ArmBase m_armBase;
    private final IntakeBase m_intakeBase;

    private final LoggedDashboardChooser<GridToRobotPose> m_startingPose = new LoggedDashboardChooser<>("AutoBuilder/StartingPose");
    private final LoggedDashboardChooser<GridDepositionLevels> m_initialDepositionLevel = new LoggedDashboardChooser<>("AutoBuilder/StartingLevel");

    private final LoggedDashboardChooser<FloorPiecePose> m_firstPiece = new LoggedDashboardChooser<>("AutoBuilder/FirstCargo");
    private final LoggedDashboardChooser<GridToRobotPose> m_firstPieceDepositionPose = new LoggedDashboardChooser<>("AutoBuilder/FirstCargoDepositionPose");
    private final LoggedDashboardChooser<GridDepositionLevels> m_firstDepositionLevel = new LoggedDashboardChooser<>("AutoBuilder/FirstLevel");

    private final LoggedDashboardChooser<FloorPiecePose> m_secondPiece = new LoggedDashboardChooser<>("AutoBuilder/SecondCargo");
    private final LoggedDashboardChooser<GridToRobotPose> m_secondPieceDepositionPose = new LoggedDashboardChooser<>("AutoBuilder/SecondCargoDepositionPose");
    private final LoggedDashboardChooser<GridDepositionLevels> m_secondDepositionLevel = new LoggedDashboardChooser<>("AutoBuilder/SecondLevel");

    private final LoggedDashboardChooser<FloorPiecePose> m_thirdPiece = new LoggedDashboardChooser<>("AutoBuilder/ThirdCargo");
    private final LoggedDashboardChooser<GridToRobotPose> m_thirdPieceDepositionPose = new LoggedDashboardChooser<>("AutoBuilder/ThirdCargoDepositionPose");
    private final LoggedDashboardChooser<GridDepositionLevels> m_thirdDepositionLevel = new LoggedDashboardChooser<>("AutoBuilder/ThirdLevel");

    private final LoggedDashboardChooser<FloorPiecePose> m_fourthPiecePose = new LoggedDashboardChooser<>("AutoBuilder/FourthCargo");
    private final LoggedDashboardChooser<GridToRobotPose> m_fourthPieceDepositionPose = new LoggedDashboardChooser<>("AutoBuilder/FourthCargoDepositionPose");
    private final LoggedDashboardChooser<GridDepositionLevels> m_fourthDepositionLevel = new LoggedDashboardChooser<>("AutoBuilder/FourthLevel");

    private final LoggedDashboardChooser<Boolean> m_levelAtEnd = new LoggedDashboardChooser<>("AutoBuilder/LevelAtEnd");

    public AutoBuilder(DriveBase driveBase, ArmBase armBase, IntakeBase intakeBase) {
        m_driveBase = driveBase;
        m_armBase = armBase;
        m_intakeBase = intakeBase;

        for(GridToRobotPose pose : GridToRobotPose.values()) {
            m_startingPose.addOption(pose.toString(), pose);

            m_firstPieceDepositionPose.addOption(pose.toString(), pose);
            m_secondPieceDepositionPose.addOption(pose.toString(), pose);
            m_thirdPieceDepositionPose.addOption(pose.toString(), pose);
            m_fourthPieceDepositionPose.addOption(pose.toString(), pose);
        }

        for(FloorPiecePose pose : FloorPiecePose.values()) {
            m_firstPiece.addOption(pose.toString(), pose);
            m_secondPiece.addOption(pose.toString(), pose);
            m_thirdPiece.addOption(pose.toString(), pose);
            m_fourthPiecePose.addOption(pose.toString(), pose);
        }

        for(GridDepositionLevels state : GridDepositionLevels.values()) {
            m_initialDepositionLevel.addOption(state.toString(), state);

            m_firstDepositionLevel.addOption(state.toString(), state);
            m_secondDepositionLevel.addOption(state.toString(), state);
            m_thirdDepositionLevel.addOption(state.toString(), state);
            m_fourthDepositionLevel.addOption(state.toString(), state);
        }

        m_levelAtEnd.addDefaultOption("No", false);
        m_levelAtEnd.addOption("Yes", true);
    }

    public Command getCommand() {
        Command autoCommand = Commands.none();

        if(m_startingPose.get() != null) {
            autoCommand = autoCommand.andThen(Commands.runOnce(() -> m_driveBase.resetPosition(m_startingPose.get().pose), m_driveBase));
        }

        autoCommand = autoCommand.andThen(new CalibrateArmExtension(m_armBase));

        if(m_initialDepositionLevel.get() != null) {
            autoCommand = autoCommand.andThen(
                    new GoToState(m_armBase, m_initialDepositionLevel.get().state.invert()),
                    new EjectIntake(m_intakeBase)
            );
        }

        if(m_firstPiece.get() != null && m_firstPieceDepositionPose != null && m_firstDepositionLevel != null) {
            Pose2d startingPose = m_startingPose.get().pose;
            Pose2d gamePiecePose = m_firstPiece.get().pose;
            Pose2d gamePieceDepositionPose = m_firstPieceDepositionPose.get().pose;
            ArmState depositionLevel = m_firstDepositionLevel.get().state;

            PathPlannerTrajectory firstPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, List.of(getPointFromPose(startingPose), getPointFromPose(gamePiecePose)));
            PathPlannerTrajectory secondPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, true, List.of(getPointFromPose(GeomUtil.invertAngle(gamePiecePose)), getPointFromPose(GeomUtil.invertAngle(gamePieceDepositionPose))));

            autoCommand = autoCommand.andThen(
                    new GoToState(m_armBase, ArmState.SCORE_HYBRID),
                    Commands.runOnce(() -> m_intakeBase.setVoltage(8), m_intakeBase),
                    new FollowTrajectory(m_driveBase, firstPath),
                    new FollowTrajectory(m_driveBase, secondPath),
                    new GoToState(m_armBase, depositionLevel.invert())
            );
        }
        if(m_secondPiece.get() != null && m_secondPieceDepositionPose != null && m_secondDepositionLevel != null) {
            Pose2d startingPose = m_firstPieceDepositionPose.get().pose;
            Pose2d gamePiecePose = m_secondPiece.get().pose;
            Pose2d gamePieceDepositionPose = m_secondPieceDepositionPose.get().pose;
            ArmState depositionLevel = m_secondDepositionLevel.get().state;

            PathPlannerTrajectory firstPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, List.of(getPointFromPose(startingPose), getPointFromPose(gamePiecePose)));
            PathPlannerTrajectory secondPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, true, List.of(getPointFromPose(GeomUtil.invertAngle(gamePiecePose)), getPointFromPose(GeomUtil.invertAngle(gamePieceDepositionPose))));

            autoCommand = autoCommand.andThen(
                    new GoToState(m_armBase, ArmState.SCORE_HYBRID),
                    Commands.runOnce(() -> m_intakeBase.setVoltage(8), m_intakeBase),
                    new FollowTrajectory(m_driveBase, firstPath),
                    new FollowTrajectory(m_driveBase, secondPath),
                    new GoToState(m_armBase, depositionLevel.invert())
            );
        }
        if(m_thirdPiece.get() != null && m_thirdPieceDepositionPose != null && m_thirdDepositionLevel != null) {
            Pose2d startingPose = m_secondPieceDepositionPose.get().pose;
            Pose2d gamePiecePose = m_thirdPiece.get().pose;
            Pose2d gamePieceDepositionPose = m_thirdPieceDepositionPose.get().pose;
            ArmState depositionLevel = m_thirdDepositionLevel.get().state;

            PathPlannerTrajectory firstPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, List.of(getPointFromPose(startingPose), getPointFromPose(gamePiecePose)));
            PathPlannerTrajectory secondPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, true, List.of(getPointFromPose(GeomUtil.invertAngle(gamePiecePose)), getPointFromPose(GeomUtil.invertAngle(gamePieceDepositionPose))));

            autoCommand = autoCommand.andThen(
                    new GoToState(m_armBase, ArmState.SCORE_HYBRID),
                    Commands.runOnce(() -> m_intakeBase.setVoltage(8), m_intakeBase),
                    new FollowTrajectory(m_driveBase, firstPath),
                    new FollowTrajectory(m_driveBase, secondPath),
                    new GoToState(m_armBase, depositionLevel.invert())
            );
        }
        if(m_fourthPiecePose.get() != null && m_fourthPieceDepositionPose != null && m_fourthDepositionLevel != null) {
            Pose2d startingPose = m_thirdPieceDepositionPose.get().pose;
            Pose2d gamePiecePose = m_fourthPiecePose.get().pose;
            Pose2d gamePieceDepositionPose = m_fourthPieceDepositionPose.get().pose;
            ArmState depositionLevel = m_fourthDepositionLevel.get().state;

            PathPlannerTrajectory firstPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, List.of(getPointFromPose(startingPose), getPointFromPose(gamePiecePose)));
            PathPlannerTrajectory secondPath = PathPlanner.generatePath(RobotLimits.kTrajectoryConstraints, true, List.of(getPointFromPose(GeomUtil.invertAngle(gamePiecePose)), getPointFromPose(GeomUtil.invertAngle(gamePieceDepositionPose))));

            autoCommand = autoCommand.andThen(
                    new GoToState(m_armBase, ArmState.SCORE_HYBRID),
                    Commands.runOnce(() -> m_intakeBase.setVoltage(8), m_intakeBase),
                    new FollowTrajectory(m_driveBase, firstPath),
                    new FollowTrajectory(m_driveBase, secondPath),
                    new GoToState(m_armBase, depositionLevel.invert())
            );
        }

        if(m_levelAtEnd.get()) {}

        return autoCommand;
    }

    enum GridToRobotPose {
        LeftLeftCone(new Pose2d(1.85, 5, new Rotation2d())),
        LeftCube(new Pose2d(1.85, 4.4, new Rotation2d())),
        LeftRightCone(new Pose2d(1.85, 3.85, new Rotation2d())),
        MiddleLeftCone(new Pose2d(1.85, 3.3, new Rotation2d())),
        MiddleCube(new Pose2d(1.85, 2.75, new Rotation2d())),
        MiddleRightCone(new Pose2d(1.85, 2.2, new Rotation2d())),
        RightLeftCone(new Pose2d(1.85, 1.6, new Rotation2d())),
        RightCube(new Pose2d(1.85, 1.07, new Rotation2d())),
        RightRightCone(new Pose2d(1.85, 0.45, new Rotation2d()));

        public final Pose2d pose;

        GridToRobotPose(Pose2d pose) {
            this.pose = pose;
        }
    }

    enum FloorPiecePose {
        Top(new Pose2d(6.17, 4.6, new Rotation2d())),
        TopMiddle(new Pose2d(6.17, 3.37, new Rotation2d())),
        BottomMiddle(new Pose2d(6.17, 2.15, new Rotation2d())),
        Bottom(new Pose2d(6.17, 0.90, new Rotation2d()));

        public final Pose2d pose;

        FloorPiecePose(Pose2d pose) {
            this.pose = pose;
        }
    }

    enum GridDepositionLevels {
        HighCube(ArmState.SCORE_HIGH_CUBE),
        MidCone(ArmState.SCORE_MID_CONE),
        MidCube(ArmState.SCORE_HIGH_CUBE),
        Hybrid(ArmState.SCORE_HYBRID);

        public final ArmState state;

        GridDepositionLevels(ArmState state) {
            this.state = state;
        }
    }

    private PathPoint getPointFromPose(Pose2d pose) {
        return new PathPoint(pose.getTranslation(), pose.getRotation());
    }
}