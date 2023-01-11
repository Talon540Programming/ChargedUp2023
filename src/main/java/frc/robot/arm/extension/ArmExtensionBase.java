package frc.robot.arm.extension;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareDevices.Arm;

/**
 * Subsystem used to extend the arm forward and backwards. Has utilities to automatically stop the arm from over-wrapping itself.
 */
public class ArmExtensionBase extends SubsystemBase {
  /**
   * Constants used to identify HAL sensors located on the Arm of the robot.
   */
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

  private ArmExtensionPosition lastExtensionPosition;

  private boolean isWinchFlipped = false;

  private final WPI_TalonFX extensionMotor =
      new WPI_TalonFX(Arm.kArmExtension.id, Arm.kArmExtension.controller);

  public ArmExtensionBase() {
    this.extensionMotor.setNeutralMode(Arm.kArmExtensionNeutralMode);

    // Do this once in initialization in case the arm is all the way down or up. // TESTME
    this.lastExtensionPosition = getCurrentPosition();
  }

  @Override
  public void periodic() {
    ArmExtensionPosition currentArmPosition = getCurrentPosition();
    // Because Unknown represents the arm moving between two points, it is considered unknown.
    if (currentArmPosition != ArmExtensionPosition.kUnknown)
      lastExtensionPosition = currentArmPosition;

    this.extensionMotor.setInverted(isWinchFlipped);

    if (currentArmPosition == ArmExtensionPosition.kStart
        || currentArmPosition == ArmExtensionPosition.kEnd)
      // Stop the motor from over-wrapping the winch
      extensionMotor.stopMotor();
  }

  /**
   * Set the percent output of the extension motor. The inversion from winch being flipped is automatically handled.
   *
   * @param percent output percent [-1, 1].
   */
  public void setExtensionPercent(double percent) {
    extensionMotor.set(ControlMode.PercentOutput, percent);
  }

  public void stopExtension() {
    extensionMotor.stopMotor();
  }

  /**
   * Get the current position of the arm. Returns unknown if none of the sensors are engaged.
   *
   * @return current position of the arm as a {@link ArmExtensionPosition}.
   */
  public ArmExtensionPosition getCurrentPosition() {
    if (ArmStartSensor.get()) {
      return ArmExtensionPosition.kStart;
    } else if (NodeOneSensor.get()) {
      return ArmExtensionPosition.kNodeOne;
    } else if (NodeTwoSensor.get()) {
      return ArmExtensionPosition.kNodeTwo;
    } else if (NodeThreeSensor.get()) {
      return ArmExtensionPosition.kNodeThree;
    } else if (ArmEndSensor.get()) {
      return ArmExtensionPosition.kEnd;
    } else {
      return ArmExtensionPosition.kUnknown;
    }
  }

  /**
   * Get the last <b>known</b> position of the extension arm. Useful if trying to optimize moving the arm to a specific position.
   *
   * @return last known position of the arm as a {@link ArmExtensionPosition}.
   */
  public ArmExtensionPosition getLastPosition() {
    return lastExtensionPosition;
  }

  /**
   * Return the state of the winch.
   *
   * @return returns true if the cable is flipped around the winch.
   */
  public boolean isWinchFlipped() {
    return isWinchFlipped;
  }

  /**
   * Set the state of the winch.
   *
   * @param flipped True if the winch is flipped and the motor control needs to be inverted.
   */
  public void setWinchFlipped(boolean flipped) {
    isWinchFlipped = flipped;
  }
}
