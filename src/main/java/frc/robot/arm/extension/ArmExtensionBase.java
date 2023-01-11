package frc.robot.arm.extension;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareDevices.Arm;

public class ArmExtensionBase extends SubsystemBase {
  public enum ArmExtensionPosition {
    kStart,
    kNodeOne,
    kNodeTwo,
    kNodeThree,
    kEnd,
    kUnknown
  }

  private final DigitalInput ArmStartSensor = new DigitalInput(Arm.kArmBasePort);
  private final DigitalInput ArmEndSensor = new DigitalInput(Arm.kArmEndPort);

  private final DigitalInput NodeOneSensor = new DigitalInput(Arm.kNodeSlotOnePort);
  private final DigitalInput NodeTwoSensor = new DigitalInput(Arm.kNodeSlotTwoPort);
  private final DigitalInput NodeThreeSensor = new DigitalInput(Arm.kNodeSlotThreePort);

  private ArmExtensionPosition lastExtensionPosition = ArmExtensionPosition.kUnknown;

  private boolean isWinchFlipped = false;

  private final WPI_TalonFX extensionMotor =
      new WPI_TalonFX(Arm.kArmExtension.id, Arm.kArmExtension.controller);

  public ArmExtensionBase() {
    this.extensionMotor.setNeutralMode(Arm.kArmExtensionNeutralMode);
  }

  @Override
  public void periodic() {
    ArmExtensionPosition currentArmPosition = getCurrentPosition();
    // Because Unknown represents the arm moving between two points, it is considered unknown.
    if (currentArmPosition != ArmExtensionPosition.kUnknown)
      lastExtensionPosition = currentArmPosition;

    this.extensionMotor.setInverted(isWinchFlipped);
  }

  public void setExtensionPercent(double percent) {
    extensionMotor.set(ControlMode.PercentOutput, percent);
  }

  public void stopExtension() {
    extensionMotor.stopMotor();
  }

  public ArmExtensionPosition getCurrentPosition() {
    if(ArmStartSensor.get()) {
      return ArmExtensionPosition.kStart;
    } else if (NodeOneSensor.get()) {
      return ArmExtensionPosition.kNodeOne;
    } else if (NodeTwoSensor.get()) {
      return ArmExtensionPosition.kNodeTwo;
    } else if (NodeThreeSensor.get()) {
      return ArmExtensionPosition.kNodeThree;
    } else if(ArmEndSensor.get()) {
      return ArmExtensionPosition.kEnd;
    } else {
      return ArmExtensionPosition.kUnknown;
    }
  }

  public ArmExtensionPosition getLastPosition() {
    return lastExtensionPosition;
  }

  public boolean isWinchFlipped() {
    return isWinchFlipped;
  }

  public void setWinchFlipped(boolean flipped) {
    isWinchFlipped = flipped;
  }
}
