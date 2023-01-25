// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.LoggerUtil;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    LoggerUtil.initializeLoggerMetadata(logger);

    // Set up data receivers & replay source
    switch (Constants.kCurrentMode) {
        // Running on a real robot, log to a USB stick
      case COMP, PROTO -> {
        logger.addDataReceiver(
            new WPILOGWriter(Constants.Logging.kLogFolders.get(Constants.kCurrentMode)));
        logger.addDataReceiver(new NT4Publisher());
      }

        // Replaying a log, set up replay source
      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    // Start AdvantageKit logger
    logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    LoggedSystemStats.getInstance().periodic();
    LoggedDriverStation.getInstance().periodic();

    switch (Constants.kCurrentMode) {
      case PROTO -> LoggedPowerDistribution.getInstance(
              HardwareDevices.PROTO.kPowerDistConfig.id, HardwareDevices.PROTO.kPowerDistType)
          .periodic();
      case COMP -> {} // TODO
      default -> LoggedPowerDistribution.getInstance().periodic();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
