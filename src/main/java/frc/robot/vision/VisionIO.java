package frc.robot.vision;

import frc.lib.logging.LoggedIO;
import frc.lib.vision.EstimatedRobotPose;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO extends LoggedIO<VisionIO.VisionIOInputs> {
  @AutoLog
  public class VisionIOInputs {
    public boolean Connected;

    public long PipelineIndex;

    public boolean DriverMode;
    public long LEDMode;

    public double CaptureTimestampSeconds;
    public double LatencyMilliseconds;
    public boolean HasTargets;

    public double[] CornerX = new double[] {};
    public double[] CornerY = new double[] {};
  }

  public String getCameraName();

  default Optional<EstimatedRobotPose> getEstimatedPose() {
    return Optional.empty();
  }
}
