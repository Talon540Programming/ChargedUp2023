package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.drivetrain.DrivetrainBase;

public class FollowTrajectory extends CommandBase {
    private final Timer m_timer = new Timer();
    private double previousTime;

    private final DrivetrainBase drivetrainBase;
    private final Trajectory targetTrajectory;
    private final boolean stopWhenDone;

    // Exists to interpolate the trajectory
    private DifferentialDriveWheelSpeeds previousWheelSpeeds;

    private final RamseteController m_ramseteController = new RamseteController(
            Constants.Drivetrain.ControlValues.Trajectory.kRamseteB,
            Constants.Drivetrain.ControlValues.Trajectory.kRamseteZeta
    );


    public FollowTrajectory(DrivetrainBase drivetrainBase, Trajectory trajectory, boolean stopWhenDone) {
        addRequirements(drivetrainBase);

        this.drivetrainBase = drivetrainBase;
        this.targetTrajectory = trajectory;
        this.stopWhenDone = stopWhenDone;
    }

    public FollowTrajectory(DrivetrainBase drivetrainBase, Trajectory trajectory) {
        this(drivetrainBase, trajectory, true);
    }

    @Override
    public void initialize() {
        // Reset the previous time to 0.
        previousTime = -1;

        // Set the last speeds to the initial speeds
        Trajectory.State initialState = targetTrajectory.sample(0);
        previousWheelSpeeds = Constants.Drivetrain.kDrivetrainKinematics.toWheelSpeeds(
                new ChassisSpeeds(
                        initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
                )
        );

        // Restart the timer
        m_timer.reset();
        m_timer.start();

        // Reset the internal PID controller values
        drivetrainBase.resetControllers();

        // Set the robot position to the initial position of the trajectory.
        drivetrainBase.resetOdometry(targetTrajectory.getInitialPose());
    }

    @Override
    public void execute() {
        double currentTime = m_timer.get();

        if(previousTime < 0) {
            // This is the first node of the trajectory.
            // Stop so that the current velocity doesn't impact later speeds. // TODO: lead previous speeds into future speeds?
            drivetrainBase.stop();
            previousTime = currentTime;
            return;
        }


        DifferentialDriveWheelSpeeds targetWheelSpeeds = Constants.Drivetrain.kDrivetrainKinematics.toWheelSpeeds(
                m_ramseteController.calculate(drivetrainBase.getRobotPosition(), targetTrajectory.sample(currentTime))
        );

        double deltaTime = currentTime - previousTime;

        // Calculate the acceleration of the robot from the last and current wheel speeds
        drivetrainBase.setFromWheelSpeeds(
                targetWheelSpeeds,
                (targetWheelSpeeds.leftMetersPerSecond - previousWheelSpeeds.leftMetersPerSecond) / deltaTime,
                (targetWheelSpeeds.rightMetersPerSecond - previousWheelSpeeds.rightMetersPerSecond) / deltaTime
        );

        previousWheelSpeeds = targetWheelSpeeds;
        previousTime = currentTime;
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(targetTrajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if(interrupted || stopWhenDone)
            drivetrainBase.stop();
    }
}
