package frc.lib.pathplanner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PathPlannerUtils {
  /**
   * Get the Valid PathPlanner trajectories in a PathPlanner friendly format.
   *
   * @return List of paths
   */
  public static List<String> getPaths() {
    Path pathPlannerPath =
        Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
    try (Stream<Path> stream = Files.walk(pathPlannerPath)) {
      return stream
          .filter(f -> f.endsWith(".path"))
          .map(
              f -> {
                String fileName = f.toString();
                return fileName.substring(0, fileName.lastIndexOf("."));
              })
          .toList();
    } catch (IOException e) {
      return Collections.emptyList();
    }
  }

  /**
   * Configure a DashboardChooser to have options for all the found paths in the deployed directory.
   * Default option is "none".
   *
   * @param trajectoryChooser DashboardChooser to modify.
   */
  public static void configureTrajectoryChooser(LoggedDashboardChooser<String> trajectoryChooser) {
    trajectoryChooser.addDefaultOption("None", "none");
    List<String> pathPlannerPaths = getPaths();

    if (pathPlannerPaths == null) {
      DriverStation.reportWarning("No Paths were found", false);
    } else {
      for (String path : pathPlannerPaths) {
        trajectoryChooser.addOption(path, path);
      }
    }
  }
}
