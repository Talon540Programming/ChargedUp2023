package frc.lib.vision;

public enum TrackingMode {
    kUnknown(-1),
    kAprilTag(0),
    kReflective(1);

    public final int pipeline;

    private TrackingMode(int pipeline) {
        this.pipeline = pipeline;
    }
}
