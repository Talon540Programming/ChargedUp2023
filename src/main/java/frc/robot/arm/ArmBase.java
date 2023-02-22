package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.sensors.encoder.QuadratureEncoderIO;
import frc.robot.sensors.encoder.QuadratureEncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs = new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs = new ArmRotationIOInputsAutoLogged();

  private final QuadratureEncoderIO m_rotationEncoderIO;
  public final QuadratureEncoderIOInputsAutoLogged m_rotationEncoderInputs = new QuadratureEncoderIOInputsAutoLogged();

  public ArmBase(
      ArmExtensionIO extensionIO, ArmRotationIO rotationIO, QuadratureEncoderIO rotationEncoderIO) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;
    this.m_rotationEncoderIO = rotationEncoderIO;
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);

    m_armRotationIO.updateInputs(m_armRotationInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    m_rotationEncoderIO.updateInputs(m_rotationEncoderInputs);
    Logger.getInstance().processInputs("Arm/Rotation/Encoder", m_rotationEncoderInputs);

    // Log the target state
    Logger.getInstance().processInputs("Arm/TargetState", ArmStateManager.getInstance().getArmState());
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
