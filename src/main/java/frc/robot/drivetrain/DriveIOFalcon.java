package frc.robot.drivetrain;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.TalonFXMechanism;
import frc.robot.constants.Constants;

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

  private final TalonFXMechanism m_leftEncoder;
  private final TalonFXMechanism m_rightEncoder;

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
      boolean leftEncoderInverted,
      boolean rightSideInverted,
      boolean rightEncoderInverted) {
    m_leftLeader = new WPI_TalonFX(leftLeaderId);
    m_leftFollower = new WPI_TalonFX(leftFollowerId);
    m_rightLeader = new WPI_TalonFX(rightLeaderId);
    m_rightFollower = new WPI_TalonFX(rightFollowerId);

    // TODO, test to make sure new TalonFXMechanism works
    m_leftEncoder =
        new TalonFXMechanism(
            m_leftLeader.getSensorCollection(),
            driveWheelRadiusMeters,
            driveGearRatio,
            leftEncoderInverted);
    m_rightEncoder =
        new TalonFXMechanism(
            m_rightLeader.getSensorCollection(),
            driveWheelRadiusMeters,
            driveGearRatio,
            rightEncoderInverted);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(
            true, 40, 0, 0);

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
    inputs.LeftPositionMeters = m_leftEncoder.getPositionMeters();
    inputs.LeftVelocityMetersPerSecond = m_leftEncoder.getVelocityMetersPerSecond();
    inputs.RightPositionMeters = m_rightEncoder.getPositionMeters();
    inputs.RightVelocityMetersPerSecond = m_rightEncoder.getVelocityMetersPerSecond();

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
    m_leftEncoder.resetPosition(0);
    m_rightEncoder.resetPosition(0);
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
