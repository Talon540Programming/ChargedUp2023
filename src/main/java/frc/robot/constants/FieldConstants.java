package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

/**
 * Constants pertaining to the Field and its dimensions. All dimensions are respective to the Blue
 * Alliance where the blue alliance is on the left wall of the field.
 */
public class FieldConstants {
  public static final AprilTagFieldLayout kFieldLayout;

  public static final double kFieldLengthInches = 651.25;
  public static final double kFieldLengthMeters = Units.inchesToMeters(kFieldLengthInches);

  public static final double kFieldWidthInches = 315.5;
  public static final double kFieldWidthMeters = Units.inchesToMeters(kFieldWidthInches);

  static {
    try {
      kFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      throw new RuntimeException("Unable to Load the AprilTagFieldLayout of the field");
    }
  }

  public static class GridNode {
    public final NodeType nodeType;

    public final Pose3d nodePose;

    public GridNode(NodeType type, Pose3d pose) {
      this.nodeType = type;
      this.nodePose = pose;
    }
  }

  public enum NodeType {
    Cone,
    Cube,
    Hybrid
  }

  public enum NodeLevel {
    High,
    Middle,
    Low
  }

  /**
   * Represents the GRID on the field. Each pose represents the point at the origin of that GRID
   * Node. (Top Center of the PEG for CONES and Center of the Square for the rest)
   */
  // spotless:off
  public static final GridNode[][] kGrid =
      new GridNode[][] {
        new GridNode[] {
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 0.508730, 1.170, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.High, new Pose3d(0.379667, 1.067530, 0.826326, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 1.626330, 1.170, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 2.185130, 1.170, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.High, new Pose3d(0.379667, 2.743930, 0.826326, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 3.302730, 1.170, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 3.861530, 1.170, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.High, new Pose3d(0.379667, 4.420330, 0.826326, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.High, new Pose3d(0.376585, 4.979130, 1.170, new Rotation3d(0, 0, 0)))
        },
        new GridNode[] {
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 0.508730, 0.865950, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.Middle, new Pose3d(0.828040, 1.067530, 0.523050, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 1.626330, 0.865950, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 2.185130, 0.865950, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.Middle, new Pose3d(0.828040, 2.743930, 0.523050, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 3.302730, 0.865950, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 3.861530, 0.865950, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cube, NodeLevel.Middle, new Pose3d(0.828040, 4.420330, 0.523050, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Cone, NodeLevel.Middle, new Pose3d(0.798040, 4.979130, 0.865950, new Rotation3d(0, 0, 0)))
        },
        new GridNode[] {
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 0.508730, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 1.067530, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 1.626330, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 2.185130, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 2.743930, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 3.302730, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 3.861530, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 4.420330, 0, new Rotation3d(0, 0, 0))),
          new GridNode(NodeType.Hybrid, NodeLevel.Low, new Pose3d(1.186813, 4.979130, 0, new Rotation3d(0, 0, 0)))
        },
      };
    // spotless:on

  public static final Pose3d kTopFloorPiecePose =
      new Pose3d(7.06, 4.58, 0, new Rotation3d(0, -Math.PI / 2, 0));
  public static final Pose3d kTopMiddleFloorPiecePose =
      new Pose3d(7.06, 3.35, 0, new Rotation3d(0, -Math.PI / 2, 0));
  public static final Pose3d kBottomMiddleFloorPiecePose =
      new Pose3d(7.06, 2.15, 0, new Rotation3d(0, -Math.PI / 2, 0));
  public static final Pose3d kBottomFloorPiecePose =
      new Pose3d(7.06, 0.90, 0, new Rotation3d(0, -Math.PI / 2, 0));

  public static final Pose3d kSingleSubstationPose =
      new Pose3d(14.20, 8, 0.75, new Rotation3d(0, Math.PI / 6, -Math.PI / 2));
  public static final Pose3d kDoubleSubstationLeftPose =
      new Pose3d(16.35, 6.957, 0.946150, new Rotation3d(0, -Math.PI / 2, 0));
  public static final Pose3d kDoubleSubstationRightPose =
      new Pose3d(16.35, 5.73, 0.946150, new Rotation3d(0, -Math.PI / 2, 0));
}
