package frc.lib.vision;


import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.Supplier;

public class PhotonCamera {
    private final org.photonvision.PhotonCamera m_camera;

    private final Supplier<Translation3d> kRobotToCamera;

    public PhotonCamera(String subtableName, Supplier<Translation3d> robotToCameraSupplier) {
        m_camera = new org.photonvision.PhotonCamera(subtableName);
        kRobotToCamera = robotToCameraSupplier;
    }

    public PhotonCamera(String subtableName, Translation3d robotToCamera) {
        this(subtableName, () -> robotToCamera);
    }



}
