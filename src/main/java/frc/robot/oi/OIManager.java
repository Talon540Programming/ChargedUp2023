package frc.robot.oi;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.HardwareDevices;

public class OIManager {
  private final DriverInterface m_driverInterface;
  private final OperatorInterface m_operatorInterface;

  public OIManager() {
    if (RobotBase.isSimulation()) {
      PS4DriverOperator controls = new PS4DriverOperator(0);
      m_driverInterface = controls;
      m_operatorInterface = controls;
    } else {
      m_driverInterface = new XboxDriver(HardwareDevices.kDriverPort);
      m_operatorInterface = new XboxOperator(HardwareDevices.kOperatorPort);
    }
  }

  public DriverInterface getDriverInterface() {
    return m_driverInterface;
  }

  public OperatorInterface getOperatorInterface() {
    return m_operatorInterface;
  }
}
