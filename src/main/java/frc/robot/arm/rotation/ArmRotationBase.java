package frc.robot.arm.rotation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmRotationBase extends SubsystemBase {
    private final ArmRotationIO m_armIO;
    public final ArmRotationIOInputsAutoLogged m_rotationInputs = new ArmRotationIOInputsAutoLogged();

    public ArmRotationBase(ArmRotationIO armIO) {
        this.m_armIO = armIO;
    }

    @Override
    public void periodic() {
        m_armIO.updateInputs(m_rotationInputs);
        Logger.getInstance().processInputs("Arm", m_rotationInputs);
    }
}
