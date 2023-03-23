package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.SparkMaxBurnManager;
import frc.lib.vision.PhotonCamera;
import frc.lib.vision.position.EstimatedRobotPose;
import frc.lib.vision.position.VisionPoseEstimator;
import frc.robot.arm.ArmBase;
import frc.robot.arm.commands.ArmControlVoltage;
import frc.robot.arm.commands.ResetArmExtension;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOSim;
import frc.robot.arm.extension.ArmExtensionIOSparkMax;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOSim;
import frc.robot.arm.rotation.ArmRotationIOSparkMax;
import frc.robot.autos.DriveDistance;
import frc.robot.autos.DriveTime;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.DriveIO;
import frc.robot.drivetrain.DriveIOFalcon;
import frc.robot.drivetrain.DriveIOSim;
import frc.robot.drivetrain.commands.DriveControl;
import frc.robot.drivetrain.commands.StabilizeRobot;
import frc.robot.intake.IntakeBase;
import frc.robot.intake.IntakeIO;
import frc.robot.intake.IntakeIOSim;
import frc.robot.intake.IntakeIOSparkMax;
import frc.robot.intake.commands.IntakeControl;
import frc.robot.oi.OIManager;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private DriveBase m_driveBase;
  private ArmBase m_armBase;
  private IntakeBase m_intakeBase;

  // Controllers
  private final OIManager m_OIManager = new OIManager();

  // PhotonCameras
  private final PhotonCamera m_forwardCamera =
      new PhotonCamera(
          HardwareDevices.kForwardCameraName, Constants.Vision.kForwardCameraTransform3d);
  private final PhotonCamera m_rearCamera =
      new PhotonCamera(HardwareDevices.kRearCameraName, Constants.Vision.kRearCameraTransform3d);

  // Trajectory Chooser
  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Autonomous Mode Chooser");

  // VisionPoseEstimator
  private final VisionPoseEstimator m_visionEstimator =
      new VisionPoseEstimator(Constants.Vision.kFieldLayout, m_forwardCamera, m_rearCamera);

  public RobotContainer() {
    SparkMaxBurnManager.checkBuildStatus();

    if (Constants.getRobotMode() != Constants.RobotMode.REPLAY) {
      switch (Constants.getRobotType()) {
        case ROBOT_2023C -> {
          m_driveBase =
              new DriveBase(
                  new DriveIOFalcon(
                      HardwareDevices.COMP2023.Drivetrain.kLeftLeaderId,
                      HardwareDevices.COMP2023.Drivetrain.kLeftFollowerId,
                      HardwareDevices.COMP2023.Drivetrain.kRightLeaderId,
                      HardwareDevices.COMP2023.Drivetrain.kRightFollowerId,
                      HardwareDevices.COMP2023.kRobotGyroId,
                      Constants.Drivetrain.kDrivetrainGearRatio,
                      Constants.Drivetrain.kWheelRadiusMeters,
                      Constants.Drivetrain.kLeftSideInverted,
                      Constants.Drivetrain.kLeftSensorInverted,
                      Constants.Drivetrain.kRightSideInverted,
                      Constants.Drivetrain.kRightSensorInverted));

          m_armBase =
              new ArmBase(
                  new ArmExtensionIOSparkMax(
                      HardwareDevices.COMP2023.Arm.kExtensionId,
                      Constants.Arm.kExtensionInverted,
                      Constants.Arm.kExtensionConversionFactor),
                  new ArmRotationIOSparkMax(
                      HardwareDevices.COMP2023.Arm.kRotationLeaderId,
                      HardwareDevices.COMP2023.Arm.kRotationFollowerId,
                      Constants.Arm.kRotationInverted,
                      HardwareDevices.COMP2023.Arm.kArmRotationEncoderId,
                      Constants.Arm.kRotationAbsoluteEncoderOffsetDegrees));

          m_intakeBase =
              new IntakeBase(
                  new IntakeIOSparkMax(
                      HardwareDevices.COMP2023.Intake.kLeftMotorId,
                      HardwareDevices.COMP2023.Intake.kRightMotorId,
                      Constants.Intake.kConversionFactor));
        }
        case ROBOT_SIMBOT -> {
          m_driveBase = new DriveBase(new DriveIOSim(false));
          m_armBase = new ArmBase(new ArmExtensionIOSim(), new ArmRotationIOSim(true));
          m_intakeBase = new IntakeBase(new IntakeIOSim());
        }
      }
    }

    // Create missing subsystems
    m_driveBase = m_driveBase != null ? m_driveBase : new DriveBase(new DriveIO() {});
    m_armBase =
        m_armBase != null
            ? m_armBase
            : new ArmBase(new ArmExtensionIO() {}, new ArmRotationIO() {});
    m_intakeBase = m_intakeBase != null ? m_intakeBase : new IntakeBase(new IntakeIO() {});

    configureBindings();
    configureAuto();
  }

  private void configureBindings() {
    m_driveBase.setDefaultCommand(new DriveControl(m_driveBase, m_OIManager.getDriverInterface()));
    m_armBase.setDefaultCommand(
        new ArmControlVoltage(m_armBase, m_OIManager.getOperatorInterface()));
    m_intakeBase.setDefaultCommand(
        new IntakeControl(m_intakeBase, m_OIManager.getOperatorInterface()));

    m_OIManager.getDriverInterface().toggleBalanceMode().whileTrue(new StabilizeRobot(m_driveBase));
    m_OIManager.getOperatorInterface().resetExtension().onTrue(new ResetArmExtension(m_armBase));
  }

  private void configureAuto() {
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Drive For 5 Seconds", new DriveTime(5, 0.25, m_driveBase));
    m_autoChooser.addOption("Drive For 5 Seconds (inverse)", new DriveTime(5, -0.25, m_driveBase));
    m_autoChooser.addOption("Drive for 2 meters", new DriveDistance(2, m_driveBase));
    m_autoChooser.addOption("Stabilize Only", new StabilizeRobot(m_driveBase));
  }

  public void pollVisionData() {
    // Log Data from the Cameras to AKit
    m_forwardCamera.logData();
    m_rearCamera.logData();

    // Log VisionPoseEstimator data
    HashMap<String, Optional<EstimatedRobotPose>> data = m_visionEstimator.getRobotPose();

    for (Map.Entry<String, Optional<EstimatedRobotPose>> entry : data.entrySet()) {
      entry
          .getValue()
          .ifPresent(
              pose -> {
                String cameraName = entry.getKey();

                Logger.getInstance()
                    .recordOutput(
                        "Vision/EstimatedPosition/" + cameraName + "/Pose", pose.robotPose);

                Logger.getInstance()
                    .recordOutput(
                        "Vision/EstimatedPosition/" + cameraName + "/Timestamp",
                        pose.timestampSeconds);

                m_driveBase.addEstimatedPose(pose.robotPose, pose.timestampSeconds);
              });
    }
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public static void configureFieldOrigin() {
    Constants.Vision.kFieldLayout.setOrigin(
        DriverStation.getAlliance() == DriverStation.Alliance.Blue
            ? AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide
            : AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
  }
}
