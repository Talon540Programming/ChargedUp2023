package frc.lib.logging;

import edu.wpi.first.wpilibj.RobotBase;
import frc.generated.BuildConstants;
import frc.robot.constants.Constants;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.littletonrobotics.junction.Logger;

/** Utilities used by the AdvantageKit Logger. */
@SuppressWarnings("DataFlowIssue")
public class LoggerUtil {
  /**
   * Initialize the Logger with the auto-generated data from the build.
   *
   * @param logger logger to update.
   */
  public static void initializeLoggerMetadata(Logger logger) {
    // Record metadata from generated state file.
    logger.recordMetadata("Robot", Constants.getRobotType().name());
    logger.recordMetadata("RuntimeEnvironment", RobotBase.getRuntimeType().name());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Set the current GIT state of the robot (helps manage the logs that are saved).
    switch (BuildConstants.DIRTY) {
      case 0 -> logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> logger.recordMetadata("GitDirty", "Uncommitted changes");
      default -> logger.recordMetadata("GitDirty", "Unknown");
    }
  }

  public static String getUSBPath() {
    // Return the path of the USB drive it is plugged in, else, return null.
    try {
      Path drivePath = Paths.get("/u").toRealPath();
      return drivePath.toString();
    } catch (IOException e) {
      return null;
    }
  }
}
