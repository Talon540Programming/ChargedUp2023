package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs =
      new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs =
      new ArmRotationIOInputsAutoLogged();

  private final ArmVisualizer m_measuredVisualizer =
      new ArmVisualizer("Measured", Constants.Arm.kArmKinematics);

  private final ArmVisualizer m_targetVisualizer =
      new ArmVisualizer("Target", Constants.Arm.kArmKinematics);

  public ArmBase(ArmExtensionIO extensionIO, ArmRotationIO rotationIO) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);

    m_armRotationIO.updateInputs(m_armRotationInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    m_measuredVisualizer.update(
        m_armRotationInputs.AbsoluteArmPositionRad, m_armExtensionInputs.DistanceTraveledMeters);

    ArmState targetState = ArmStateManager.getInstance().getTargetState();

    // Log the target state
    Logger.getInstance().processInputs("Arm/TargetState", targetState);

    m_targetVisualizer.update(targetState.AngleRadians, targetState.ArmLengthMeters);
  }

  /**
   * Set the voltage output of the rotation motors.
   *
   * @param voltage voltage to set.
   */
  public void setRotationVoltage(double voltage) {
    m_armRotationIO.setVoltage(voltage);
  }

  /**
   * Set the voltage output of the extension motors.
   *
   * @param voltage voltage to set.
   */
  public void setExtensionVoltage(double voltage) {
    m_armExtensionIO.setVoltage(voltage);
  }
}
