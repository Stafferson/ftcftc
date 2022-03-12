package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    public CANSparkMax motor;

    public IndexerSubsystem() {
        motor = new CANSparkMax(10, MotorType.kBrushless);
        motor.setInverted(false);
    }

    public void spin() {
        motor.set(1.0);
    }

    public void stop() {
        motor.set(0.0);
    }
}
