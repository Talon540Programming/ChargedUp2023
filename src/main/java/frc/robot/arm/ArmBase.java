package frc.robot.arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.sensors.BeamBreakSensor;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs = new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs = new ArmRotationIOInputsAutoLogged();

  private final BeamBreakSensor m_forwardBeamBreak, m_reverseBeamBreak;
  private final DigitalInput m_homeSensor;

  public ArmBase(
      ArmExtensionIO extensionIO, ArmRotationIO rotationIO, int forwardPort, int reversePort, int homeSensorPort) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;

    this.m_forwardBeamBreak = new BeamBreakSensor(forwardPort);
    this.m_reverseBeamBreak = new BeamBreakSensor(reversePort);
    this.m_homeSensor = new DigitalInput(homeSensorPort);
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);

    m_armRotationIO.updateInputs(m_armRotationInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    Logger.getInstance().recordOutput("Arm/Forward Beam Break", atForwardLimit());
    Logger.getInstance().recordOutput("Arm/Reverse Beam Break", atRearLimit());
    Logger.getInstance().recordOutput("Arm/HOME HALL Sensor", extensionAtHome());
  }

  public boolean extensionAtHome() {
    return m_homeSensor.get();
  }

  public boolean atForwardLimit() {
    return m_forwardBeamBreak.isBeamBroken();
  }

  public boolean atRearLimit() {
    return m_reverseBeamBreak.isBeamBroken();
  }
}
