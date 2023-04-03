package frc.robot.drivetrain.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.Constants;
import frc.robot.drivetrain.DriveBase;

public class FollowTrajectory extends PPRamseteCommand {
  public FollowTrajectory(DriveBase driveBase, PathPlannerTrajectory trajectory) {
    super(
        trajectory,
        driveBase::getPosition,
        new RamseteController(),
        new SimpleMotorFeedforward(
            Constants.Drivetrain.ControlValues.Characterization.kSLinear,
            Constants.Drivetrain.ControlValues.Characterization.kVLinear,
            Constants.Drivetrain.ControlValues.Characterization.kALinear),
        Constants.Drivetrain.kDrivetrainKinematics,
        driveBase::getWheelSpeeds,
        new PIDController(
            Constants.Drivetrain.ControlValues.Trajectory.kP,
            Constants.Drivetrain.ControlValues.Trajectory.kI,
            Constants.Drivetrain.ControlValues.Trajectory.kD),
        new PIDController(
            Constants.Drivetrain.ControlValues.Trajectory.kP,
            Constants.Drivetrain.ControlValues.Trajectory.kI,
            Constants.Drivetrain.ControlValues.Trajectory.kD),
        driveBase::tankDriveVoltage,
        true,
        driveBase);
  }
}
