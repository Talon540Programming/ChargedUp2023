package frc.lib.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An object that can be used to track or represent the state of, or desired state of the arm.
 */
public class ArmState {
    public Rotation2d angle;
    public double extensionLengthMeters;

    /**
     * Create an ArmState object.
     *
     * @param angle Angle of the arm. This angle should be similar to the Unit circle where 0 means the arm is perpendicular to the robot's fulcrum and parallel to the floor, facing towards the front of the robot. The angle is CCW positive.
     * @param extensionLengthMeters The length that the arm is at without the grabber or any other attachment at the end in meters.
     */
    public ArmState(Rotation2d angle, double extensionLengthMeters) {
        this.angle = angle;
        this.extensionLengthMeters = extensionLengthMeters;
    }

    /**
     * Create an ArmState object.
     *
     * @param angleRadians Angle of the Arm in radians. This angle should be similar to the Unit circle where 0 means the arm is perpendicular to the robot's fulcrum and parallel to the floor, facing towards the front of the robot. The angle is CCW positive.
     * @param extensionLengthMeters The length that the arm is at without the grabber or any other attachment at the end in meters.
     */
    public ArmState(double angleRadians, double extensionLengthMeters) {
        this(Rotation2d.fromRadians(angleRadians), extensionLengthMeters);
    }
}
