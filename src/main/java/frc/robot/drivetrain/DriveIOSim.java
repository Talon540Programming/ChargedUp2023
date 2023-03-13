package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;

public class DriveIOSim implements DriveIO {
  private final DifferentialDrivetrainSim m_driveSim;

  public DriveIOSim() {
    double batteryMOI = Units.lbsToKilograms(12.5) * Math.pow(Units.inchesToMeters(5), 2);
    double gearboxMOI =
        (1219.9 / 1000) * Math.pow(RobotDimensions.Drivetrain.kDrivetrainLengthMeters / 2.0, 2);

    m_driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            Constants.Drivetrain.kDrivetrainGearRatio,
            batteryMOI + gearboxMOI,
            RobotDimensions.kRobotMassKilos,
            Constants.Drivetrain.kWheelRadiusMeters,
            Constants.Drivetrain.kTrackWidthMeters,
            null);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    m_driveSim.update(Constants.loopPeriodSecs);

    inputs.LeftPositionMeters =
        m_driveSim.getLeftPositionMeters() / Constants.Drivetrain.kWheelRadiusMeters;
    inputs.LeftVelocityMetersPerSecond =
        m_driveSim.getLeftVelocityMetersPerSecond() / Constants.Drivetrain.kWheelRadiusMeters;
    inputs.RightPositionMeters =
        m_driveSim.getRightPositionMeters() / Constants.Drivetrain.kWheelRadiusMeters;
    inputs.RightVelocityMetersPerSecond =
        m_driveSim.getRightVelocityMetersPerSecond() / Constants.Drivetrain.kWheelRadiusMeters;

    inputs.CurrentAmps =
        new double[] {m_driveSim.getLeftCurrentDrawAmps(), m_driveSim.getRightCurrentDrawAmps()};

    inputs.GyroYawRad = -m_driveSim.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    m_driveSim.setInputs(leftVolts, rightVolts);
  }

  @Override
  public Rotation2d getHeading() {
    return m_driveSim.getHeading();
  }
}
