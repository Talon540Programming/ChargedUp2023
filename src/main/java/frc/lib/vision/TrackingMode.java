package frc.lib.vision;

public enum TrackingMode {
  kUnknown(-1),
  kAprilTag(0), // TODO
  kReflective(1); // TODO

  public final int pipeline;

  private TrackingMode(int pipeline) {
    this.pipeline = pipeline;
  }
}
