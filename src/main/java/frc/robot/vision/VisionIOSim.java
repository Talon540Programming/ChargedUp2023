package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.SimVisionSystem;

public class VisionIOSim extends VisionIOPhotonCamera {
  private final SimVisionSystem m_visionSim;
  private final Supplier<Pose2d> m_robotPoseSupplier;

  public VisionIOSim(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    super(cameraName, robotToCamera);

    m_visionSim = new SimVisionSystem(cameraName, 75, robotToCamera, 5, 640, 480, 10);

    m_visionSim.addVisionTargets(FieldConstants.kFieldLayout);

    m_robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSim.processFrame(m_robotPoseSupplier.get());

    super.updateInputs(inputs);
  }
}
