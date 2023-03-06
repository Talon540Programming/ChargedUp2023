package frc.robot.arm.extension;

public class Winch {
  private final double initialSpoolRadiusMeters;
  private final double totalCableLengthMeters;
  private final double cableDiameterMeters;
  private final int wrapsPerRadiusIncrease;
  private final double initialNumberOfWraps;

  public Winch(
      double initialSpoolRadiusMeters,
      double totalCableLengthMeters,
      double cableDiameterMeters,
      int wrapsPerRadiusIncrease,
      double numberOfWraps) {
    this.initialSpoolRadiusMeters = initialSpoolRadiusMeters;
    this.totalCableLengthMeters = totalCableLengthMeters;
    this.cableDiameterMeters = cableDiameterMeters;
    this.wrapsPerRadiusIncrease = wrapsPerRadiusIncrease;
    this.initialNumberOfWraps = numberOfWraps;
  }

  /**
   * Get the distance from the winch to the end of the cable.
   *
   * @param winchRevolutions number of rotations of the winch.
   * @return distance from winch to end of cable.
   */
  public double getDistanceTraveled(double winchRevolutions) {
    winchRevolutions += initialNumberOfWraps;

    // Total length of the cable - Distance wrapped around the winch
    double winchDistance = 0;

    int numberOfRadiusIncreases = (int) (winchRevolutions / wrapsPerRadiusIncrease);
    double remainingRevolutions = winchRevolutions % wrapsPerRadiusIncrease;

    for (int i = 0; i < numberOfRadiusIncreases; i++) {
      winchDistance +=
          2
              * Math.PI
              * (initialSpoolRadiusMeters + (i * cableDiameterMeters))
              * wrapsPerRadiusIncrease;
    }

    winchDistance +=
        2
            * Math.PI
            * (initialSpoolRadiusMeters + (numberOfRadiusIncreases * cableDiameterMeters))
            * remainingRevolutions;

    return totalCableLengthMeters - winchDistance;
  }
}
