package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.pathplanner.PathPlannerUtils;
import frc.lib.vision.EstimatedRobotPose;
import frc.lib.vision.PhotonCamera;
import frc.lib.vision.VisionPoseEstimator;
import frc.robot.arm.ArmBase;
import frc.robot.arm.commands.StateController;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIONeo;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIONeo;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.DriveIO;
import frc.robot.drivetrain.DriveIOFalcon;
import frc.robot.drivetrain.commands.StabilizeRobot;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import frc.robot.sensors.encoder.QuadratureEncoderIO;
import frc.robot.sensors.encoder.QuadratureEncoderIOCANCoder;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.gyro.GyroIOPigeon2;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.talon540.control.XboxController.TalonXboxController;

public class RobotContainer {
  // Subsystems
  private final DriveBase m_driveBase;
  private final ArmBase m_armBase;

  // Controllers
  private final TalonXboxController m_driverController =
      new TalonXboxController(HardwareDevices.kDriverXboxControllerPort);
  private final TalonXboxController m_depositionController =
      new TalonXboxController(HardwareDevices.kDepositionXboxControllerPort);

  // PhotonCameras
  private final PhotonCamera m_forwardCamera =
      new PhotonCamera(
          HardwareDevices.kForwardCameraName, Constants.Vision.kForwardCameraTransform3d);
  private final PhotonCamera m_rearCamera =
      new PhotonCamera(HardwareDevices.kRearCameraName, Constants.Vision.kRearCameraTransform3d);

  // Trajectory Chooser
  private final LoggedDashboardChooser<String> m_trajectoryChooser =
      new LoggedDashboardChooser<>("Trajectory Chooser");

  // VisionPoseEstimator
  private final VisionPoseEstimator m_visionEstimator =
      new VisionPoseEstimator(Constants.Vision.kFieldLayout, m_forwardCamera, m_rearCamera);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    DriveIO driveIO;
    GyroIO gyroIO;
    ArmExtensionIO extensionIO;
    ArmRotationIO rotationIO;
    QuadratureEncoderIO armRotationEncoderIO;

    if (Constants.getRobotMode() == Constants.RobotMode.REAL) {
      switch (Constants.getRobotType()) {
        case ROBOT_2023C -> {
          driveIO = new DriveIO() {};
          gyroIO = new GyroIO() {};
          extensionIO = new ArmExtensionIO() {};
          rotationIO = new ArmRotationIO() {};
          armRotationEncoderIO = new QuadratureEncoderIO() {};
        }
        case ROBOT_2023P -> {
          driveIO =
              new DriveIOFalcon(
                  HardwareDevices.PROTO2023.Drivetrain.kLeftLeader,
                  HardwareDevices.PROTO2023.Drivetrain.kLeftFollower,
                  HardwareDevices.PROTO2023.Drivetrain.kRightLeader,
                  HardwareDevices.PROTO2023.Drivetrain.kRightFollower,
                  Constants.Drivetrain.kDrivetrainGearRatio,
                  Constants.Drivetrain.kWheelRadiusMeters,
                  Constants.Drivetrain.kLeftSideInverted,
                  Constants.Drivetrain.kLeftSensorInverted,
                  Constants.Drivetrain.kRightSideInverted,
                  Constants.Drivetrain.kRightSensorInverted);
          gyroIO = new GyroIOPigeon2(HardwareDevices.PROTO2023.kRobotGyroConfig);
          extensionIO =
              new ArmExtensionIONeo(
                  HardwareDevices.PROTO2023.Arm.kExtension,
                  Constants.Arm.kExtensionInverted,
                  Constants.Arm.kExtensionEncoderInverted);
          rotationIO =
              new ArmRotationIONeo(
                  HardwareDevices.PROTO2023.Arm.kRotationLeader,
                  HardwareDevices.PROTO2023.Arm.kRotationFollower,
                  Constants.Arm.kRotationInverted);

          armRotationEncoderIO =
              new QuadratureEncoderIOCANCoder(HardwareDevices.PROTO2023.Arm.kArmRotationEncoder);
        }
        default -> throw new RuntimeException("Unknown Robot Type");
      }
    } else {
      driveIO = new DriveIO() {};
      gyroIO = new GyroIO() {};
      extensionIO = new ArmExtensionIO() {};
      rotationIO = new ArmRotationIO() {};
      armRotationEncoderIO = new QuadratureEncoderIO() {};
    }

    m_driveBase = new DriveBase(driveIO, gyroIO);
    m_armBase = new ArmBase(extensionIO, rotationIO, armRotationEncoderIO);

    configureBindings();

    PathPlannerUtils.configureTrajectoryChooser(m_trajectoryChooser);
  }

  private void configureBindings() {
    m_driveBase.setDefaultCommand(new XboxControllerDriveControl(m_driveBase, m_driverController));
    m_armBase.setDefaultCommand(new StateController(m_armBase));

    m_driverController.leftBumper().whileTrue(new StabilizeRobot(m_driveBase));

    // By controlling manually with commands, the StateController is de-scheduled which will bypass
    // control to the controller (manual).
    new Trigger(() -> Math.abs(m_depositionController.getLeftY()) < 0.05)
        .whileTrue(
            new RunCommand(
                () -> m_armBase.setRotationVoltage(m_depositionController.getLeftDeadbandY() * 12),
                m_armBase));
    new Trigger(() -> Math.abs(m_depositionController.getRightY()) < 0.05)
        .whileTrue(
            new RunCommand(
                () ->
                    m_armBase.setExtensionVoltage(m_depositionController.getRightDeadbandY() * 12),
                m_armBase));
  }

  public void pollVisionData() {
    Logger.getInstance().processInputs(m_forwardCamera.getName(), m_forwardCamera);
    Logger.getInstance().processInputs(m_rearCamera.getName(), m_rearCamera);

    HashMap<String, Optional<EstimatedRobotPose>> data = m_visionEstimator.getRobotPose();

    for (Map.Entry<String, Optional<EstimatedRobotPose>> entry : data.entrySet()) {
      Optional<EstimatedRobotPose> value = entry.getValue();
      if (value.isPresent()) {
        EstimatedRobotPose pose = value.get();
        m_driveBase.addEstimatedPose(pose.robotPose, pose.timestampSeconds);
      }
    }
  }

  public Command getAutonomousCommand() {
    String path = m_trajectoryChooser.get();

    if (!path.equals("none")) {
      PathPlannerTrajectory m_trajectory =
          PathPlanner.loadPath(
              path,
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
    } else {
      DriverStation.reportError("You tried to run Auto, but no path was selected.", false);
      return null;
    }
  }
}
