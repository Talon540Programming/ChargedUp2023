package frc.robot.arm.rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareDevices.Arm;

public class ArmRotationBase extends SubsystemBase {
  private final WPI_TalonFX rotationLeader;
  private final WPI_TalonFX rotationFollower;

  private final WPI_CANCoder rotationEncoder;

  private final DigitalInput forwardBeamBreak = new DigitalInput(0);
  private final DigitalInput rearBeamBreak = new DigitalInput(0);

  public ArmRotationBase() {
    // Configure rotation motors
    this.rotationLeader =
        new WPI_TalonFX(Arm.kArmRotationLeader.id, Arm.kArmRotationLeader.controller);
    this.rotationFollower =
        new WPI_TalonFX(Arm.kArmRotationFollower.id, Arm.kArmRotationFollower.controller);

    this.rotationLeader.setNeutralMode(Arm.kArmRotationNeutralMode);
    this.rotationFollower.setNeutralMode(Arm.kArmRotationNeutralMode);

    this.rotationFollower.follow(rotationLeader);

    // Configure rotation encoder
    this.rotationEncoder =
        new WPI_CANCoder(Arm.kArmRotationEncoder.id, Arm.kArmRotationEncoder.controller);
  }

  public void setRotationPercent(double percent) {
    rotationLeader.set(ControlMode.PercentOutput, percent);
  }

  public double getArmPosition() {
    return rotationEncoder.getAbsolutePosition();
  }

  public boolean isAtForwardLimit() {
    return forwardBeamBreak.get();
  }

  public boolean isAtRearLimit() {
    return rearBeamBreak.get();
  }
}
