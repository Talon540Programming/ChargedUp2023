package frc.lib.trajectory;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

public class PathPlannerUtil {
  private PathPlannerUtil() {}

  public static List<String> getDeployedPaths() {
    Path pathPlannerPath =
        Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
    try (Stream<Path> stream = Files.walk(pathPlannerPath)) {
      return stream
          .filter(f -> f.toString().strip().endsWith(".path"))
          .map(f -> f.getFileName().toString().split("\\.")[0])
          .toList();
    } catch (IOException e) {
      return Collections.emptyList();
    }
  }
}
