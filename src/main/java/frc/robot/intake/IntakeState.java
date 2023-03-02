package frc.robot.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeState implements LoggableInputs {
    public double WristAngleRad;
    public double ClawAngleRad;

    public IntakeState(double clawAngleRad, double wristAngleRad) {
        this.WristAngleRad = wristAngleRad;
        this.ClawAngleRad = clawAngleRad;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("WristAngleRad", WristAngleRad);
        table.put("ClawAngleRad", ClawAngleRad);
    }

    @Override
    public void fromLog(LogTable table) {
        WristAngleRad = table.getDouble("WristAngleRad", WristAngleRad);
        ClawAngleRad = table.getDouble("ClawAngleRad", ClawAngleRad);
    }
}
