package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;

public class DriveIOSim implements DriveIO {
  private final DifferentialDrivetrainSim m_driveSim;

  public DriveIOSim() {
    double batteryMOI = Units.lbsToKilograms(12.5) * Math.pow(Units.inchesToMeters(5), 2);
    double gearboxMOI =
        (1219.9 / 1000) * Math.pow(RobotDimensions.Drivetrain.kDrivetrainLengthMeters / 2.0, 2);

    // m_driveSim =
    //     new DifferentialDrivetrainSim(
    //         DCMotor.getFalcon500(2),
    //         Constants.Drivetrain.kDrivetrainGearRatio,
    //         batteryMOI + gearboxMOI,
    //         RobotDimensions.kRobotMassKilos,
    //         Constants.Drivetrain.kWheelRadiusMeters,
    //         Constants.Drivetrain.kTrackWidthMeters,
    //         null);

    m_driveSim =
        DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleFalcon500PerSide,
            KitbotGearing.k12p75,
            KitbotWheelSize.kSixInch,
            null);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    m_driveSim.update(Constants.loopPeriodSecs);

    inputs.LeftPositionMeters = m_driveSim.getLeftPositionMeters();
    inputs.LeftVelocityMetersPerSecond = m_driveSim.getLeftVelocityMetersPerSecond();
    inputs.RightPositionMeters = m_driveSim.getRightPositionMeters();
    inputs.RightVelocityMetersPerSecond = m_driveSim.getRightVelocityMetersPerSecond();

    inputs.CurrentAmps =
        new double[] {m_driveSim.getLeftCurrentDrawAmps(), m_driveSim.getRightCurrentDrawAmps()};

    inputs.GyroYawRad = -m_driveSim.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    SmartDashboard.putNumber("left percent", leftVolts);
    SmartDashboard.putNumber("right percent", rightVolts);

    m_driveSim.setInputs(leftVolts, rightVolts);
  }

  @Override
  public Rotation2d getHeading() {
    return m_driveSim.getHeading();
  }
}
