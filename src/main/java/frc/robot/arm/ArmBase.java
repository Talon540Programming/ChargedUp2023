package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.extension.Winch;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs =
      new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs =
      new ArmRotationIOInputsAutoLogged();

  private final Winch m_extensionWinch;

  Mechanism2d m_mech = new Mechanism2d(
    Units.inchesToMeters(96), Units.inchesToMeters(78));

  MechanismRoot2d m_mechFulcrum = m_mech.getRoot(
    "ArmFulcrum", Units.inchesToMeters(48), RobotDimensions.Arm.kFulcrumHeightMeters);


  MechanismLigament2d m_mechUpright = 
  m_mechFulcrum.append(new MechanismLigament2d("ArmTower", RobotDimensions.Arm.kFulcrumHeightMeters, -90));

  private final MechanismLigament2d m_mechArm =
      m_mechFulcrum.append(
        new MechanismLigament2d(
            "Arm",
            25 / 39.37,
            Units.radiansToDegrees(m_armRotationInputs.AbsoluteArmPositionRad),
            6,
            new Color8Bit(Color.kYellow)));

  public ArmBase(ArmExtensionIO extensionIO, ArmRotationIO rotationIO) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;

    this.m_extensionWinch =
        new Winch(
            Constants.Arm.kExtensionWinchRadiusMeters,
            Constants.Arm.kExtensionCableLengthMeters,
            Constants.Arm.kExtensionCableDiameterMeters,
            Constants.Arm.kNumberOfWrapsPerRadiusIncrease,
            Constants.Arm.kInitialWrapsAtBoot);

    SmartDashboard.putData("Arm Simulation", m_mech);
    m_mechUpright.setColor(new Color8Bit(Color.kAqua));
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);

    Logger.getInstance()
        .recordOutput("Arm/Extension/Winch/Distance", getExtensionDistanceTraveled());

    m_armRotationIO.updateInputs(m_armRotationInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    m_mechArm.setAngle(Rotation2d.fromRadians(m_armRotationInputs.AbsoluteArmPositionRad));

    // Log the target state
    Logger.getInstance()
        .processInputs("Arm/TargetState", ArmStateManager.getInstance().getTargetState());
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

  /**
   * Get the distance from the winch to the end of the cable.
   *
   * @return distance traveled by the arm.
   */
  public double getExtensionDistanceTraveled() {
    return m_extensionWinch.getDistanceTraveled(m_armExtensionInputs.DistanceTraveledMeters);
  }
}
