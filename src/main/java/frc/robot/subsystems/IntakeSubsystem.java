package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public CANSparkMax motor;

    public IntakeSubsystem() {
        motor = new CANSparkMax(9, MotorType.kBrushless);
    }

    public void spin(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0.0);
    }
}
