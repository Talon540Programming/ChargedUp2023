// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.DriveIO;
import frc.robot.drivetrain.DriveIOFalcon;
import frc.robot.drivetrain.commands.StabilizeRobot;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.gyro.GyroIOPigeon2;
import org.talon540.control.XboxController.TalonXboxController;

public class RobotContainer {
  // Subsystems
  private final DriveBase m_driveBase;

  // Controllers
  private final TalonXboxController m_driverController =
      new TalonXboxController(HardwareDevices.kDriverXboxControllerPort);
  private final TalonXboxController m_depositionController =
      new TalonXboxController(HardwareDevices.kDepositionXboxControllerPort);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    PathPlannerServer.startServer(5811);

    DriveIO driveIO;
    GyroIO gyroIO;

    if (Constants.getRobotMode() == Constants.RobotMode.REAL) {
      switch (Constants.getRobotType()) {
        case ROBOT_2023C -> {
          driveIO = new DriveIO() {};
          gyroIO = new GyroIO() {};
        }
        case ROBOT_2023P -> {
          driveIO =
              new DriveIOFalcon(
                  new DriveIOFalcon.DriveIOFalconConfig(
                      HardwareDevices.PROTO2023.Drivetrain.kLeftLeader,
                      HardwareDevices.PROTO2023.Drivetrain.kLeftFollower,
                      HardwareDevices.PROTO2023.Drivetrain.kRightLeader,
                      HardwareDevices.PROTO2023.Drivetrain.kRightFollower,
                      Constants.Drivetrain.kDrivetrainGearRatio,
                      Constants.Drivetrain.kWheelRadiusMeters,
                      Constants.Drivetrain.kLeftSideInverted,
                      Constants.Drivetrain.kLeftSensorInverted,
                      Constants.Drivetrain.kRightSideInverted,
                      Constants.Drivetrain.kRightSensorInverted));
          gyroIO =
              new GyroIOPigeon2(
                  new GyroIOPigeon2.GyroIOPigeon2Config(
                      HardwareDevices.PROTO2023.kRobotGyroConfig));
        }
        default -> throw new RuntimeException("Unknown Robot Type");
      }
    } else {
      driveIO = new DriveIO() {};
      gyroIO = new GyroIO() {};
    }

    m_driveBase = new DriveBase(driveIO, gyroIO);

    configureBindings();
  }

  private void configureBindings() {
    // Configure Driver Controller
    m_driveBase.setDefaultCommand(new XboxControllerDriveControl(m_driveBase, m_driverController));

    m_driverController.leftBumper().onTrue(new StabilizeRobot(m_driveBase));
  }

  public Command getAutonomousCommand() {
    PathPlannerTrajectory m_trajectory =
        PathPlanner.loadPath(
            "TestPath",
            Constants.Drivetrain.kMaxDrivetrainVelocityMetersPerSecond,
            Constants.Drivetrain.kMaxDrivetrainAccelerationMetersPerSecondSquared);

    m_driveBase.resetPosition(m_trajectory.getInitialPose());

    return new PPRamseteCommand(
            m_trajectory,
            m_driveBase::getPosition,
            new RamseteController(
                Constants.Drivetrain.ControlValues.Trajectory.kRamseteB,
                Constants.Drivetrain.ControlValues.Trajectory.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.Drivetrain.ControlValues.WheelSpeed.kS,
                Constants.Drivetrain.ControlValues.WheelSpeed.kV,
                Constants.Drivetrain.ControlValues.WheelSpeed.kA),
            Constants.Drivetrain.kDrivetrainKinematics,
            m_driveBase::getWheelSpeeds,
            new PIDController(
                Constants.Drivetrain.ControlValues.WheelSpeed.kP,
                Constants.Drivetrain.ControlValues.WheelSpeed.kI,
                Constants.Drivetrain.ControlValues.WheelSpeed.kD),
            new PIDController(
                Constants.Drivetrain.ControlValues.WheelSpeed.kP,
                Constants.Drivetrain.ControlValues.WheelSpeed.kI,
                Constants.Drivetrain.ControlValues.WheelSpeed.kD),
            m_driveBase::tankDriveVoltage,
            true,
            m_driveBase)
        .andThen(m_driveBase::stop);
  }
}
