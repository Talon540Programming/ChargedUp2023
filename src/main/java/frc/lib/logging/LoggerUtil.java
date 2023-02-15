package frc.lib.logging;

import edu.wpi.first.wpilibj.RobotBase;
import frc.generated.BuildConstants;
import frc.robot.constants.Constants;
import java.io.IOException;
import java.nio.file.Path;
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
    logger.recordMetadata("ROBOT_NAME", Constants.getRobotType().toString());
    logger.recordMetadata("RUNTIME_ENVIRONMENT", RobotBase.getRuntimeType().toString());
    logger.recordMetadata("PROJECT_NAME", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BUILD_DATE", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GIT_DATE", BuildConstants.GIT_DATE);
    logger.recordMetadata("GIT_BRANCH", BuildConstants.GIT_BRANCH);

    // Set the current GIT state of the robot (helps manage the logs that are saved).
    switch (BuildConstants.DIRTY) {
      case 0 -> logger.recordMetadata("GIT_STATUS", "All changes committed");
      case 1 -> logger.recordMetadata("GIT_STATUS", "Uncommitted changes");
      default -> logger.recordMetadata("GIT_STATUS", "Unknown");
    }
  }

  /**
   * Get the path of the USB drive if it is plugged into the roboRIO. If one isn't found, it will
   * return null.
   *
   * @return path of the USB drive.
   */
  public static String getUSBPath() {
    // Return the path of the USB drive it is plugged in, else, return null.
    try {
      Path drivePath = Path.of("/u").toRealPath();
      return drivePath.toString();
    } catch (IOException e) {
      return null;
    }
  }
}
