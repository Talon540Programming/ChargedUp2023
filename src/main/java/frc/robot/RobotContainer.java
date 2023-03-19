package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmBase;
import frc.robot.arm.commands.ArmControl;
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
import frc.robot.oi.OIManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private DriveBase m_driveBase;
  private ArmBase m_armBase;

  // Controllers
  private final OIManager m_manager = new OIManager();

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
        case ROBOT_SIMBOT -> {
          m_driveBase = new DriveBase(new DriveIOSim(false));
          m_armBase = new ArmBase(new ArmExtensionIOSim(), new ArmRotationIOSim(true));
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
    m_driveBase.setDefaultCommand(new DriveControl(m_driveBase, m_manager.getDriverInterface()));
    m_armBase.setDefaultCommand(new ArmControl(m_armBase, m_manager.getOperatorInterface()));

    m_manager.getDriverInterface().toggleBalanceMode().whileTrue(new StabilizeRobot(m_driveBase));
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
