package frc.lib.vision;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.Supplier;

public class PhotonCamera extends org.photonvision.PhotonCamera {
    private final Supplier<Transform3d> kRobotToCamera;

    public PhotonCamera(String subtableName, Supplier<Transform3d> robotToCameraSupplier) {
        super(subtableName);
            kRobotToCamera = robotToCameraSupplier;
    }

    public PhotonCamera(String subtableName, Transform3d robotToCamera) {
        this(subtableName, () -> robotToCamera);
    }

    public Transform3d getRobotToCamera() {
        return kRobotToCamera.get();
    }
}
