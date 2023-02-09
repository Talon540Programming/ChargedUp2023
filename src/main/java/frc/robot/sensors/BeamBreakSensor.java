package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor {
  private final DigitalInput m_sensor;

  public BeamBreakSensor(int receiverPort) {
    this.m_sensor = new DigitalInput(receiverPort);
  }

  public boolean isBeamBroken() {
    return !m_sensor.get();
  }
}
