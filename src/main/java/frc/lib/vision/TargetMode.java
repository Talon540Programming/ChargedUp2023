package frc.lib.vision;

public enum TargetMode {
  kUnknown(-1),
  kAprilTag(0), // TODO
  kReflective(1); // TODO

  public final int pipeline;

  private TargetMode(int pipeline) {
    this.pipeline = pipeline;
  }
}
