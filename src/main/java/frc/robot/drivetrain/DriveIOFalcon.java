package frc.robot.drivetrain;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import org.talon540.sensors.TalonFXMechanism;

/**
 * {@link DriveIO} used for interfacing with a series of {@link WPI_TalonFX} devices for a
 * Differential Drivetrain.
 */
public class DriveIOFalcon implements DriveIO {
  private final WPI_TalonFX m_leftLeader;
  private final WPI_TalonFX m_rightLeader;
  private final WPI_TalonFX m_leftFollower;
  private final WPI_TalonFX m_rightFollower;

  private final WPI_Pigeon2 m_gyro;
  private final double[] xyzDegreesPerSecond = new double[3];
  private final short[] xyzAccelData = new short[3];

  private final TalonFXMechanism m_leftSensors;
  private final TalonFXMechanism m_rightSensors;

  private final boolean leftSensorInverted;
  private final boolean rightSensorInverted;

  /** Create the DriveIO. */
  public DriveIOFalcon(
      int leftLeaderId,
      int leftFollowerId,
      int rightLeaderId,
      int rightFollowerId,
      int gyroId,
      double driveGearRatio,
      double driveWheelRadiusMeters,
      boolean leftSideInverted,
      boolean leftSensorInverted,
      boolean rightSideInverted,
      boolean rightSensorInverted) {
    this.leftSensorInverted = leftSensorInverted;
    this.rightSensorInverted = rightSensorInverted;

    m_leftLeader = new WPI_TalonFX(leftLeaderId);
    m_leftFollower = new WPI_TalonFX(leftFollowerId);
    m_rightLeader = new WPI_TalonFX(rightLeaderId);
    m_rightFollower = new WPI_TalonFX(rightFollowerId);

    m_leftSensors =
        new TalonFXMechanism(
            m_leftLeader.getSensorCollection(), driveWheelRadiusMeters, driveGearRatio);

    m_rightSensors =
        new TalonFXMechanism(
            m_rightLeader.getSensorCollection(), driveWheelRadiusMeters, driveGearRatio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);

    m_leftLeader.configAllSettings(config);
    m_rightLeader.configAllSettings(config);

    setNeutralMode(Constants.Drivetrain.kDrivetrainDefaultNeutralMode);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(leftSideInverted);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightLeader.setInverted(rightSideInverted);
    m_rightFollower.setInverted(InvertType.FollowMaster);

    m_gyro = new WPI_Pigeon2(gyroId);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    double leftSignum = leftSensorInverted ? -1 : 1;
    double rightSignum = rightSensorInverted ? -1 : 1;

    inputs.LeftPositionMeters = leftSignum * m_leftSensors.getPosition();
    inputs.LeftVelocityMetersPerSecond = leftSignum * m_leftSensors.getLinearVelocity();
    inputs.RightPositionMeters = rightSignum * m_rightSensors.getPosition();
    inputs.RightVelocityMetersPerSecond = rightSignum * m_rightSensors.getLinearVelocity();

    inputs.TemperatureCelsius =
        new double[] {
          m_leftLeader.getTemperature(), m_leftFollower.getTemperature(),
          m_rightLeader.getTemperature(), m_rightFollower.getTemperature()
        };

    inputs.CurrentAmps =
        new double[] {
          m_leftLeader.getSupplyCurrent(), m_leftFollower.getSupplyCurrent(),
          m_rightLeader.getSupplyCurrent(), m_rightFollower.getSupplyCurrent()
        };

    // Handle Gyro Inputs
    m_gyro.getRawGyro(xyzDegreesPerSecond);
    m_gyro.getBiasedAccelerometer(xyzAccelData);

    inputs.GyroConnected = m_gyro.getLastError().equals(ErrorCode.OK);

    inputs.YawPositionRad = Math.toRadians(m_gyro.getYaw());
    inputs.PitchPositionRad = Math.toRadians(m_gyro.getPitch());
    inputs.RollPositionRad = Math.toRadians(m_gyro.getRoll());

    inputs.YawRateRadPerSecond = Math.toRadians(xyzDegreesPerSecond[2]);
    inputs.PitchRateRadPerSecond = Math.toRadians(xyzDegreesPerSecond[1]);
    inputs.RollRateRadPerSecond = Math.toRadians(xyzDegreesPerSecond[0]);

    inputs.AccelXGForces = (double) xyzAccelData[0] / (1 << 14);
    inputs.AccelYGForces = (double) xyzAccelData[1] / (1 << 14);
    inputs.AccelZGForces = (double) xyzAccelData[2] / (1 << 14);
  }

  @Override
  public void resetEncoders() {
    m_leftSensors.resetEncoder();
    m_rightSensors.resetEncoder();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leftLeader.setNeutralMode(mode.toPhoenixMode());
    m_rightLeader.setNeutralMode(mode.toPhoenixMode());
    m_leftFollower.setNeutralMode(mode.toPhoenixMode());
    m_rightFollower.setNeutralMode(mode.toPhoenixMode());
  }

  @Override
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void resetHeading() {
    m_gyro.reset();
  }
}
