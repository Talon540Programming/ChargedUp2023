package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.commands.CalibrateArmExtension;
import frc.robot.arm.commands.GoToState;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;
import frc.robot.drivetrain.DriveBase;
import frc.robot.groups.AutoBalance;
import frc.robot.intake.IntakeBase;
import frc.robot.intake.commands.EjectIntake;
import java.util.List;
import java.util.Map;

public class AutoFactory {
  private final DriveBase m_driveBase;
  private final ArmBase m_armBase;
  private final IntakeBase m_intakeBase;

  private final Map<String, Command> m_eventMap;

  public AutoFactory(DriveBase driveBase, ArmBase armBase, IntakeBase intakeBase) {
    m_driveBase = driveBase;
    m_armBase = armBase;
    m_intakeBase = intakeBase;

    m_eventMap =
        Map.ofEntries(
            // Intake Commands
            Map.entry(
                "StartIntakeIn", Commands.runOnce(() -> m_intakeBase.setVoltage(8), m_intakeBase)),
            Map.entry(
                "StartIntakeOut",
                Commands.runOnce(() -> m_intakeBase.setVoltage(-8), m_intakeBase)),
            Map.entry("StopIntake", Commands.runOnce(m_intakeBase::stop, m_intakeBase)),
            Map.entry("EjectIntake", new EjectIntake(m_intakeBase)),

            // Start Going To Commands
            Map.entry(
                "StartArmFloor",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.FLOOR), m_armBase)),
            Map.entry(
                "StartArmHybrid",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.SCORE_HYBRID), m_armBase)),
            Map.entry(
                "StartArmConeMid",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.SCORE_MID_CONE), m_armBase)),
            Map.entry(
                "StartArmCubeMid",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.SCORE_MID_CUBE), m_armBase)),
            Map.entry(
                "StartArmCubeHigh",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.SCORE_HIGH_CUBE), m_armBase)),
            Map.entry(
                "StartArmFloorInvert",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.FLOOR.invert()), m_armBase)),
            Map.entry(
                "StartArmHybridInvert",
                Commands.runOnce(
                    () -> m_armBase.updateState(ArmState.SCORE_HYBRID.invert()), m_armBase)),
            Map.entry(
                "StartArmConeMidInvert",
                Commands.runOnce(
                    () -> m_armBase.updateState(ArmState.SCORE_MID_CONE.invert()), m_armBase)),
            Map.entry(
                "StartArmCubeMidInvert",
                Commands.runOnce(
                    () -> m_armBase.updateState(ArmState.SCORE_MID_CUBE.invert()), m_armBase)),
            Map.entry(
                "StartArmCubeHighInvert",
                Commands.runOnce(
                    () -> m_armBase.updateState(ArmState.SCORE_HIGH_CUBE.invert()), m_armBase)),
            Map.entry(
                "StartArmIdle",
                Commands.runOnce(() -> m_armBase.updateState(ArmState.FLOOR), m_armBase)),

            // Go To Commands
            Map.entry("GoToFloor", new GoToState(m_armBase, ArmState.FLOOR)),
            Map.entry("GoToHybrid", new GoToState(m_armBase, ArmState.SCORE_HYBRID)),
            Map.entry("GoToConeMid", new GoToState(m_armBase, ArmState.SCORE_MID_CONE)),
            Map.entry("GoToCubeMid", new GoToState(m_armBase, ArmState.SCORE_MID_CUBE)),
            Map.entry("GoToCubeHigh", new GoToState(m_armBase, ArmState.SCORE_HIGH_CUBE)),
            Map.entry("GoToFloorInvert", new GoToState(m_armBase, ArmState.FLOOR.invert())),
            Map.entry("GoToHybridInvert", new GoToState(m_armBase, ArmState.SCORE_HYBRID.invert())),
            Map.entry(
                "GoToConeMidInvert", new GoToState(m_armBase, ArmState.SCORE_MID_CONE.invert())),
            Map.entry(
                "GoToCubeMidInvert", new GoToState(m_armBase, ArmState.SCORE_MID_CUBE.invert())),
            Map.entry(
                "GoToCubeHighInvert", new GoToState(m_armBase, ArmState.SCORE_HIGH_CUBE.invert())),
            Map.entry("GoToIdle", new GoToState(m_armBase, ArmState.IDLE)),
            Map.entry("AutoBalance", new AutoBalance(m_driveBase, m_armBase, false)),
            Map.entry("AutoBalanceEnd", new AutoBalance(m_driveBase, m_armBase, true)));
  }

  public CommandBase createAutoCommandFromPaths(List<PathPlannerTrajectory> paths) {
    RamseteAutoBuilder autoBuilder =
        new RamseteAutoBuilder(
            m_driveBase::getPosition,
            m_driveBase::resetPosition,
            new RamseteController(),
            Constants.Drivetrain.kDrivetrainKinematics,
            new SimpleMotorFeedforward(
                Constants.Drivetrain.ControlValues.Characterization.kSLinear,
                Constants.Drivetrain.ControlValues.Characterization.kVLinear,
                Constants.Drivetrain.ControlValues.Characterization.kALinear),
            m_driveBase::getWheelSpeeds,
            new PIDConstants(
                Constants.Drivetrain.ControlValues.Trajectory.kP,
                Constants.Drivetrain.ControlValues.Trajectory.kI,
                Constants.Drivetrain.ControlValues.Trajectory.kD),
            m_driveBase::tankDriveVoltage,
            m_eventMap,
            true,
            m_driveBase);

    return autoBuilder
        .fullAuto(paths)
        .beforeStarting(
            Commands.either(
                Commands.none(),
                new CalibrateArmExtension(m_armBase),
                m_armBase::extensionCalibrated));
  }

  public CommandBase createAutoCommandFromPaths(String pathGroupName) {
    return createAutoCommandFromPaths(
        PathPlanner.loadPathGroup(pathGroupName, RobotLimits.kTrajectoryConstraints));
  }
}
