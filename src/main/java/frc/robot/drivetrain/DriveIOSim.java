package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.constants.Constants;

public class DriveIOSim implements DriveIO {
  private final DifferentialDrivetrainSim m_driveSim;

  public DriveIOSim(boolean simulateSystemNoise) {
    m_driveSim =
        new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(
            Constants.Drivetrain.ControlValues.Characterization.kVLinear,
            Constants.Drivetrain.ControlValues.Characterization.kALinear,
            Constants.Drivetrain.ControlValues.Characterization.kVAngular,
            Constants.Drivetrain.ControlValues.Characterization.kAAngular),
          DCMotor.getFalcon500(2),
          Constants.Drivetrain.kDrivetrainGearRatio,
          Constants.Drivetrain.kTrackWidthMeters,
          Constants.Drivetrain.kWheelRadiusMeters,
          simulateSystemNoise 
          ? VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005) 
          : null 
    );

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

    inputs.GyroYawRad = -m_driveSim.getHeading().getRadians(); // TODO: CCP CCWP
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
