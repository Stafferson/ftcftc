package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax motor;

    public TurretSubsystem() {
        motor = new CANSparkMax(10, MotorType.kBrushless);
        motor.setInverted(true);
    }

    public void spin(double speed) {
        motor.set(0.1 * speed); 
    }

    public void stop() {
        motor.set(0.0);
    }
}
