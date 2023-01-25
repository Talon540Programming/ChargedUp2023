package frc.lib.logging;

import frc.robot.BuildConstants;
import org.littletonrobotics.junction.Logger;

public class LoggerUtil {
    public static void initializeLoggerMetadata(Logger logger) {
        // Record metadata from generated state file.
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
}
