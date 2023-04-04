package frc.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;

public class AllianceGeometryUtil {
    private AllianceGeometryUtil() {}

    public static Pose2d apply(Pose2d pose) {
        if(shouldFlip()) {
            return new Pose2d(
                    apply(pose.getTranslation()),
                    apply(pose.getRotation())
            );
        } else {
            return pose;
        }
    }

    public static Pose3d apply(Pose3d pose) {
        if(shouldFlip()) {
            return new Pose3d(
                    apply(pose.getTranslation()),
                    apply(pose.getRotation())
            );
        } else {
            return pose;
        }
    }

    public static Translation2d apply(Translation2d translation) {
        if(shouldFlip()) {
            return new Translation2d(FieldConstants.kFieldLengthMeters - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    public static Translation3d apply(Translation3d translation) {
        if(shouldFlip()) {
            return new Translation3d(FieldConstants.kFieldLengthMeters - translation.getX(), translation.getY(), translation.getZ());
        } else {
            return translation;
        }
    }

    public static Rotation2d apply(Rotation2d rotation) {
        if(shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    public static Rotation3d apply(Rotation3d rotation) {
        if(shouldFlip()) {
            double angle;

            double x = -Math.cos(rotation.getZ());
            double y = Math.sin(rotation.getZ());

            double magnitude = Math.hypot(x, y);
            if (magnitude > 1e-6) {
                angle = Math.atan2(y / magnitude, x / magnitude);
            } else {
                angle = Math.atan2(0.0, 1.0);
            }

            return new Rotation3d(rotation.getX(), rotation.getY(), angle);
        } else {
            return rotation;
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red;
    }
}
