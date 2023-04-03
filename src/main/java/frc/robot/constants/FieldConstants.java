package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Constants pertaining to the Field and its dimensions. All dimensions are respective to the Blue
 * Alliance where the blue alliance is on the left wall of the field.
 */
public class FieldConstants {
  public static class GridNode {
    public final GamePieceType type;

    public final Pose3d requiredEffectorPoseForDeposition;

    public GridNode(GamePieceType type, Pose3d pose) {
      this.type = type;
      this.requiredEffectorPoseForDeposition = pose;
    }

    enum GamePieceType {
      Cone,
      Cube,
      Hybrid
    }
  }

  /**
   * Represents the GRID on the field. Each pose represents the point at the origin of that GRID
   * Node. (Top Center of the PEG for CONES and Center of the Square for the rest)
   */
  // spotless:off
  public static final GridNode[][] grid =
      new GridNode[][] {
        new GridNode[] {
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 0.508730, 1.170, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.379667, 1.067530, 0.826326, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 1.626330, 1.170, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 2.185130, 1.170, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.379667, 2.743930, 0.826326, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 3.302730, 1.170, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 3.861530, 1.170, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.379667, 4.420330, 0.826326, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.376585, 4.979130, 1.170, new Rotation3d(0, -Math.PI / 2, 0)))
        },
        new GridNode[] {
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 0.508730, 0.865950, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.828040, 1.067530, 0.523050, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 1.626330, 0.865950, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 2.185130, 0.865950, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.828040, 2.743930, 0.523050, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 3.302730, 0.865950, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 3.861530, 0.865950, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cube, new Pose3d(0.828040, 4.420330, 0.523050, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Cone, new Pose3d(0.798040, 4.979130, 0.865950, new Rotation3d(0, -Math.PI / 2, 0)))
        },
        new GridNode[] {
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 0.508730, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 1.067530, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 1.626330, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 2.185130, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 2.743930, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 3.302730, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 3.861530, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 4.420330, 0, new Rotation3d(0, -Math.PI / 2, 0))),
          new GridNode(GridNode.GamePieceType.Hybrid, new Pose3d(1.186813, 4.979130, 0, new Rotation3d(0, -Math.PI / 2, 0)))
        },
      };
    // spotless:on
}
