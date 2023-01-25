package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.Constants.Drivetrain.ControlValues;
import frc.robot.constants.Constants;
import frc.robot.drivetrain.DriveBase;

public class WPIFollowTrajectory extends RamseteCommand {
  public WPIFollowTrajectory(DriveBase driveBase, Trajectory trajectory) {
    super(
            trajectory,
            driveBase::getPose,
            new RamseteController(ControlValues.Trajectory.kRamseteB, ControlValues.Trajectory.kRamseteZeta),
            new SimpleMotorFeedforward(ControlValues.WheelSpeed.kS, ControlValues.WheelSpeed.kV, ControlValues.WheelSpeed.kA),
            Constants.Drivetrain.kDrivetrainKinematics,
            driveBase::getWheelSpeeds,
            new PIDController(ControlValues.WheelSpeed.kP, ControlValues.WheelSpeed.kI, ControlValues.WheelSpeed.kD),
            new PIDController(ControlValues.WheelSpeed.kP, ControlValues.WheelSpeed.kI, ControlValues.WheelSpeed.kD),
            driveBase::tankDriveVoltage,
            driveBase
    );
  }
}

