package frc.robot.arm.rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareDevices.Arm;

/** Subsystem used to manage the rotation aspect of the arm. */
public class ArmRotationBase extends SubsystemBase {
  private final WPI_TalonFX rotationLeader = new WPI_TalonFX(Arm.Rotation.kArmRotationLeader.id, Arm.Rotation.kArmRotationLeader.controller);
  private final WPI_TalonFX rotationFollower = new WPI_TalonFX(Arm.Rotation.kArmRotationFollower.id, Arm.Rotation.kArmRotationFollower.controller);

  private final WPI_CANCoder rotationEncoder = new WPI_CANCoder(Arm.Rotation.kArmRotationEncoder.id, Arm.Rotation.kArmRotationEncoder.controller);

  private final DigitalInput forwardBeamBreak = new DigitalInput(Arm.Rotation.kForwardBeamBreakPort);
  private final DigitalInput rearBeamBreak = new DigitalInput(Arm.Rotation.kRearBeamBreakPort);

  public ArmRotationBase() {
    // Configure rotation motors
    this.rotationLeader.setNeutralMode(Arm.Rotation.kArmRotationNeutralMode);
    this.rotationFollower.setNeutralMode(Arm.Rotation.kArmRotationNeutralMode);

    this.rotationFollower.follow(rotationLeader);
  }

  /**
   * Set the output percent of the arm rotation motor. Automatically sets the second motor with
   * follower mode.
   *
   * @param percent percent to set [-1, 1].
   */
  public void setRotationPercent(double percent) {
    rotationLeader.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Return the position of the rotation encoder.
   *
   * @return position of the rotation encoder.
   */
  public double getArmPosition() {
    return rotationEncoder.getAbsolutePosition();
  }

  /**
   * Return if the forward beam break sensor is broken. Useful for stopping the arm from forcing
   * itself past this point and damaging itself.
   *
   * @return if the forward beam break sensor is true.
   */
  public boolean isAtForwardLimit() {
    return forwardBeamBreak.get();
  }

  /**
   * Return if the rear beam break sensor is broken. Useful for stopping the arm from forcing itself
   * past this point and damaging itself.
   *
   * @return if the rear beam break sensor is true.
   */
  public boolean isAtRearLimit() {
    return rearBeamBreak.get();
  }
}
