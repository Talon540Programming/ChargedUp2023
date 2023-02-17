package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.arm.ArmState;
import frc.lib.arm.ArmStateManager;
import frc.lib.arm.ArmUtil;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs =
      new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs =
      new ArmRotationIOInputsAutoLogged();

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

    // Get the target Arm State
    ArmState targetState = ArmStateManager.getArmState();

    // Log the target state
    Logger.getInstance().processInputs("Arm/TargetState", targetState);
  }

  public void setRotationVoltage(double voltage) {
    m_armRotationIO.setVoltage(voltage);
  }

  public void setExtensionVoltage(double voltage) {
    m_armExtensionIO.setVoltage(voltage);
  }

  public double getArmLengthMeters() {
    return ArmUtil.calculateArmLength(m_armExtensionInputs.DistanceTraveledMeters);
  }
}
