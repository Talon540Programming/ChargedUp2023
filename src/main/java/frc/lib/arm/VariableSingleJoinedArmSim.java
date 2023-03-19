package frc.lib.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.arm.ArmKinematics;
import frc.robot.constants.RobotDimensions;

/**
 * Represents a Sim of a Single Joined Arm that has variable length and center of mass. Credit WPI
 * and Contributors for source.
 */
public class VariableSingleJoinedArmSim extends LinearSystemSim<N2, N1, N1> {
  private final DCMotor m_gearbox;
  private final double m_gearing;

  private double m_armLenMeters;
  private double m_effectorLengthMeters;

  private final ArmKinematics m_kinematics;

  private final boolean m_simulateGravity;

  /**
   * Creates a simulated arm mechanism.
   *
   * @param plant The linear system that represents the arm.
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param armLengthMeters The length of the arm.
   * @param armKinematics kinematics of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   */
  public VariableSingleJoinedArmSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double armLengthMeters,
      double effectorLengthMeters,
      ArmKinematics armKinematics,
      boolean simulateGravity) {
    this(plant, gearbox, gearing, armLengthMeters, effectorLengthMeters, armKinematics, simulateGravity, null);
  }

  /**
   * Creates a simulated arm mechanism.
   *
   * @param plant The linear system that represents the arm.
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param armLengthMeters The length of the arm.
   * @param armKinematics kinematics of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public VariableSingleJoinedArmSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double armLengthMeters,
      double effectorLengthMeters,
      ArmKinematics armKinematics,
      boolean simulateGravity,
      Matrix<N1, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_armLenMeters = armLengthMeters;
    m_effectorLengthMeters = effectorLengthMeters;
    m_kinematics = armKinematics;
    m_simulateGravity = simulateGravity;
  }

  /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm, can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param armKinematics kinematics of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   */
  public VariableSingleJoinedArmSim(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      double armLengthMeters,
      double effectorLengthMeters,
      ArmKinematics armKinematics,
      boolean simulateGravity) {
    this(gearbox, gearing, jKgMetersSquared, armLengthMeters,effectorLengthMeters, armKinematics, simulateGravity, null);
  }

  /**
   * Creates a simulated arm mechanism.
   *
   * @param gearbox The type of and number of motors in the arm gearbox.
   * @param gearing The gearing of the arm (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the arm; can be calculated from CAD software.
   * @param armLengthMeters The length of the arm.
   * @param armKinematics kinematics of the arm.
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public VariableSingleJoinedArmSim(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      double armLengthMeters,
      double effectorLengthMeters,
      ArmKinematics armKinematics,
      boolean simulateGravity,
      Matrix<N1, N1> measurementStdDevs) {
    super(
        LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing),
        measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    m_armLenMeters = armLengthMeters;
    m_effectorLengthMeters = effectorLengthMeters;
    m_kinematics = armKinematics;
    m_simulateGravity = simulateGravity;
  }

  /**
   * Returns the current arm angle.
   *
   * @return The current arm angle.
   */
  public double getAngleRads() {
    return m_y.get(0, 0);
  }

  /**
   * Returns the current arm velocity.
   *
   * @return The current arm velocity.
   */
  public double getVelocityRadPerSec() {
    return m_x.get(1, 0);
  }

  /**
   * Returns the arm current draw.
   *
   * @return The aram current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    var motorVelocity = m_x.get(1, 0) * m_gearing;
    return m_gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the arm.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }

  /**
   * Update the length of the arm.
   *
   * @param armLengthMeters length of the arm in meters
   */
  public void updateArmLength(double armLengthMeters) {
    this.m_armLenMeters = armLengthMeters;
  }

  public void updateEffectorLength(double effectorLengthMeters) {
    this.m_effectorLengthMeters = effectorLengthMeters;
  }

  /**
   * Update the Moment of Inertia of the System (A) and Input (B) Matrix's of the Linear System.
   *
   * @param jKgMetersSquared moment of inertia of the arm.
   */
  public void updateMoI(double jKgMetersSquared) {
    // Update System Matrix
    m_plant
        .getA()
        .set(
            1,
            1,
            -Math.pow(m_gearing, 2)
                * m_gearbox.KtNMPerAmp
                / (m_gearbox.KvRadPerSecPerVolt * m_gearbox.rOhms * jKgMetersSquared));

    // Update Input Matrix
    m_plant
        .getB()
        .set(1, 0, m_gearing * m_gearbox.KtNMPerAmp / (m_gearbox.rOhms * jKgMetersSquared));
  }

  /**
   * Updates the state of the arm.
   *
   * @param currentXhat The current state estimate.
   * @param u The system inputs (voltage).
   * @param dtSeconds The time difference between controller updates.
   */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // The torque on the arm is given by τ = F⋅r, where F is the force applied by
    // gravity and r the distance from pivot to center of mass. Recall from
    // dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
    // torque on the arm, J is the mass-moment of inertia about the pivot axis,
    // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
    //
    // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/J
    //
    // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
    // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
    // rod rotating about it's end, where L is the overall rod length. The mass
    // distribution is assumed to be uniform. Substitute r=L/2 to find:
    //
    //   α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
    //   α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
    //   α = 3/2⋅g⋅cos(θ)/L
    //
    // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
    //
    //   f(x, u) = Ax + Bu + [0  α]ᵀ
    //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ

    Matrix<N2, N1> updatedXhat =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(x.get(0, 0)) / m_armLenMeters;
                xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collision after updating xhat
    if (m_kinematics.wouldIntersectForward(m_armLenMeters + m_effectorLengthMeters, updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_kinematics.lowestForwardAngle(m_armLenMeters + m_effectorLengthMeters), 0);
    }
    if (m_kinematics.wouldIntersectRear(m_armLenMeters + m_effectorLengthMeters, updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_kinematics.lowestRearAngle(m_armLenMeters + m_effectorLengthMeters), 0);
    }
    return updatedXhat;
  }
}
