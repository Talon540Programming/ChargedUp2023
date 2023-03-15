package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.arm.ArmBase;
import frc.robot.arm.commands.ArmStateController;
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
import frc.robot.drivetrain.commands.StabilizeRobot;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.talon540.control.XboxController.TalonXboxController;

public class RobotContainer {
  // Subsystems
  private DriveBase m_driveBase;
  private ArmBase m_armBase;

  // Controllers
  private final TalonXboxController m_driverController =
      new TalonXboxController(HardwareDevices.kDriverXboxControllerPort);
  private final TalonXboxController m_depositionController =
      new TalonXboxController(HardwareDevices.kDepositionXboxControllerPort);

  // Trajectory Chooser
  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("Autonomous Mode Chooser");

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

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
                      Constants.Arm.kExtensionPositionConversionFactor,
                      Constants.Arm.kExtensionVelocityConversionFactor),
                  new ArmRotationIOSparkMax(
                      HardwareDevices.COMP2023.Arm.kRotationLeaderId,
                      HardwareDevices.COMP2023.Arm.kRotationFollowerId,
                      Constants.Arm.kRotationInverted,
                      HardwareDevices.COMP2023.Arm.kArmRotationEncoderId,
                      Constants.Arm.kRotationAbsoluteEncoderOffsetDegrees));
        }
        case ROBOT_2023P -> {
          m_driveBase =
              new DriveBase(
                  new DriveIOFalcon(
                      HardwareDevices.PROTO2023.Drivetrain.kLeftLeaderId,
                      HardwareDevices.PROTO2023.Drivetrain.kLeftFollowerId,
                      HardwareDevices.PROTO2023.Drivetrain.kRightLeaderId,
                      HardwareDevices.PROTO2023.Drivetrain.kRightFollowerId,
                      HardwareDevices.PROTO2023.kRobotGyroId,
                      Constants.Drivetrain.kDrivetrainGearRatio,
                      Constants.Drivetrain.kWheelRadiusMeters,
                      Constants.Drivetrain.kLeftSideInverted,
                      Constants.Drivetrain.kLeftSensorInverted,
                      Constants.Drivetrain.kRightSideInverted,
                      Constants.Drivetrain.kRightSensorInverted));
        }
        case ROBOT_SIMBOT -> {
          m_driveBase = new DriveBase(new DriveIOSim(false));
          m_armBase = new ArmBase(
            new ArmExtensionIOSim(), new ArmRotationIOSim(true));
        }
      }
    }

    // Create missing subsystems

    m_driveBase = m_driveBase != null ? m_driveBase : new DriveBase(new DriveIO() {});
    m_armBase =
        m_armBase != null
            ? m_armBase
            : new ArmBase(new ArmExtensionIO() {}, new ArmRotationIO() {});

    configureBindings();
    configureAuto();
  }

  private void configureBindings() {
    m_driveBase.setDefaultCommand(new XboxControllerDriveControl(m_driveBase, m_driverController));
    m_armBase.setDefaultCommand(new ArmStateController(m_armBase));

    m_driverController.leftBumper().whileTrue(new StabilizeRobot(m_driveBase));

    // By controlling manually with commands, the ArmStateController is de-scheduled which will
    // bypass control to the controller (manual).
    new Trigger(() -> Math.abs(m_depositionController.getLeftY()) < 0.05)
        .whileTrue(
            Commands.run(
                () -> m_armBase.setRotationVoltage(m_depositionController.getLeftDeadbandY() * 12),
                m_armBase));
    new Trigger(() -> Math.abs(m_depositionController.getRightY()) < 0.05)
        .whileTrue(
            Commands.run(
                () ->
                    m_armBase.setExtensionVoltage(m_depositionController.getRightDeadbandY() * 12),
                m_armBase));
  }

  private void configureAuto() {
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Drive For 5 Seconds", new DriveTime(5, 0.25, m_driveBase));
    m_autoChooser.addOption("Drive For 5 Seconds (inverse)", new DriveTime(5, -0.25, m_driveBase));

    m_autoChooser.addOption("Drive for 2 meters", new DriveDistance(2, m_driveBase));

    m_autoChooser.addOption("Stabilize Only", new StabilizeRobot(m_driveBase));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
