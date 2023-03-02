package frc.robot.sensors.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/** Object representing the Accelerometer on a Pigeon2. */
public class Pigeon2Accelerometer implements Accelerometer {
  public enum Axis {
    kX(0),
    kY(1),
    kZ(2);

    public final int axis;

    Axis(int axis) {
      this.axis = axis;
    }
  }

  private final Pigeon2 m_gyro;

  /**
   * Create a Pigeon2.0 Accelerometer object.
   *
   * @param gyro gyro to use as accelerometer.
   */
  public Pigeon2Accelerometer(Pigeon2 gyro) {
    m_gyro = gyro;
  }

  @Override
  public void setRange(Range range) {
    // Exists to satisfy WPI interface
  }

  @Override
  public double getX() {
    return getAccelData(Axis.kX.axis);
  }

  @Override
  public double getY() {
    return getAccelData(Axis.kY.axis);
  }

  @Override
  public double getZ() {
    return getAccelData(Axis.kZ.axis);
  }

  private short[] getRawAccelData() {
    short[] data = new short[3];
    m_gyro.getBiasedAccelerometer(data);
    return data;
  }

  private short getRawAccelData(int axis) {
    return getRawAccelData()[axis];
  }

  private double getAccelData(int axis) {
    return convertToActual(getRawAccelData(axis));
  }

  private double convertToActual(short raw) {
    return fixedToDouble(raw);
  }

  private double fixedToDouble(short fixed) {
    return ((double) fixed) / (1 << 14);
  }
}
