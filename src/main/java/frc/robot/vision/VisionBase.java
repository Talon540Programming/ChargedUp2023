package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class VisionBase extends SubsystemBase {
  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_ioInputs;

  private final BiConsumer<Pose2d, Double> m_poseConsumer;

  public VisionBase(BiConsumer<Pose2d, Double> poseConsumer, VisionIO... io) {
    m_poseConsumer = poseConsumer;

    m_io = new VisionIO[io.length];
    m_ioInputs = new VisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      m_io[i] = io[i];
      m_ioInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_io.length; i++) {
      m_io[i].updateInputs(m_ioInputs[i]);
      Logger.getInstance().processInputs("Vision/" + m_io[i].getCameraName(), m_ioInputs[i]);

      m_io[i]
          .getEstimatedPose()
          .ifPresent(
              pose -> m_poseConsumer.accept(pose.robotPose.toPose2d(), pose.timestampSeconds));
    }
  }
}
