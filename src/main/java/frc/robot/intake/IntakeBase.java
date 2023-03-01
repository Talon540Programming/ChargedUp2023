package frc.robot.intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.intake.claw.IntakeClawIO;
import frc.robot.intake.claw.IntakeClawInputsAutoLogged;
import frc.robot.intake.wrist.IntakeWristIO;
import frc.robot.intake.wrist.IntakeWristInputsAutoLogged;
import frc.robot.sensors.colorsensor.ColorSensorIO;
import frc.robot.sensors.colorsensor.ColorSensorIOInputsAutoLogged;
import frc.robot.sensors.encoder.QuadratureEncoderIO;
import frc.robot.sensors.encoder.QuadratureEncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase {
  private final IntakeClawIO m_clawIO;
  public final IntakeClawInputsAutoLogged m_clawInputs = new IntakeClawInputsAutoLogged();

  private final IntakeWristIO m_wristIO;
  public final IntakeWristInputsAutoLogged m_wristInputs = new IntakeWristInputsAutoLogged();

  private final QuadratureEncoderIO m_wristEncoderIO;
  public final QuadratureEncoderIOInputsAutoLogged m_wristEncoderInputs =
      new QuadratureEncoderIOInputsAutoLogged();

  private final QuadratureEncoderIO m_clawEncoderIO;
  public final QuadratureEncoderIOInputsAutoLogged m_clawEncoderInputs =
      new QuadratureEncoderIOInputsAutoLogged();

  private final ColorSensorIO m_colorSensorIO;
  public final ColorSensorIOInputsAutoLogged m_colorSensorInputs =
      new ColorSensorIOInputsAutoLogged();

  public IntakeBase(
      IntakeClawIO clawIO,
      IntakeWristIO wristIO,
      QuadratureEncoderIO wristEncoderIO,
      QuadratureEncoderIO clawEncoderIO,
      ColorSensorIO colorSensorIO) {
    this.m_clawIO = clawIO;
    this.m_wristIO = wristIO;
    this.m_wristEncoderIO = wristEncoderIO;
    m_clawEncoderIO = clawEncoderIO;
    this.m_colorSensorIO = colorSensorIO;
  }

  @Override
  public void periodic() {
    m_clawIO.updateInputs(m_clawInputs);
    Logger.getInstance().processInputs("Intake/Claw", m_clawInputs);

    m_clawEncoderIO.updateInputs(m_clawEncoderInputs);
    Logger.getInstance().processInputs("Intake/Claw/AbsoluteEncoder", m_clawEncoderInputs);

    m_wristIO.updateInputs(m_wristInputs);
    Logger.getInstance().processInputs("Intake/Wrist", m_wristInputs);

    m_wristEncoderIO.updateInputs(m_wristEncoderInputs);
    Logger.getInstance().processInputs("Intake/Wrist/AbsoluteEncoder", m_wristEncoderInputs);

    m_colorSensorIO.updateInputs(m_colorSensorInputs);
    Logger.getInstance().processInputs("Intake", m_colorSensorInputs);
  }

  public void setClawVoltage(double voltage) {
    m_clawIO.setVoltage(voltage);
  }

  public void setWristVoltage(double voltage) {
    m_wristIO.setVoltage(voltage);
  }

  public CurrentSeenTarget getCurrentSeenTarget() {
    Color8Bit currentColor = m_colorSensorIO.getColor8Bit();

    for (Constants.GamePiece piece : Constants.GamePiece.values()) {
      if (piece.matches(currentColor))
        return CurrentSeenTarget.fromGamePiece(piece);
    }

    return CurrentSeenTarget.Unknown;
  }

  public boolean isHoldingCone() {
    if (getCurrentSeenTarget() != CurrentSeenTarget.Cone) return false;

    return m_colorSensorInputs.ProximityValue < 0.6
        && m_clawEncoderInputs.AbsolutePositionRad + Math.toRadians(0.05)
            > Constants.Intake.kConeIntakeAngle;
  }

  public boolean isHoldingCube() {
    if (getCurrentSeenTarget() != CurrentSeenTarget.Cube) return false;

    return m_colorSensorInputs.ProximityValue < 0.6
        && m_clawEncoderInputs.AbsolutePositionRad + Math.toRadians(0.05)
            > Constants.Intake.kCubeIntakeAngle;
  }

  public boolean isHoldingSomething() {
    return isHoldingCube() || isHoldingCone();
  }

  public void stopClaw() {
    m_clawIO.setVoltage(0);
  }

  public void stopWrist() {
    m_wristIO.setVoltage(0);
  }

  public enum CurrentSeenTarget {
    Cone,
    Cube,
    Unknown;

    public static CurrentSeenTarget fromGamePiece(Constants.GamePiece piece) {
      return switch (piece) {
        case Cone -> Cone;
        case Cube -> Cube;
      };
    }
  }
}
