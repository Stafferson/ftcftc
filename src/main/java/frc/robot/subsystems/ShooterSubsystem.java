package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public CANSparkMax motor1, motor2;

    public ShooterSubsystem() {
        motor1 = new CANSparkMax(11, MotorType.kBrushless);
        motor2 = new CANSparkMax(12, MotorType.kBrushless);
    }

    public void spin(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    public void spit() {
        motor1.set(-1.0);
        motor2.set(-1.0);
    }

    public void idle() {
        motor1.set(0.05);
        motor2.set(0.05);
    }

    public void stop() {
        motor1.set(0.0);
        motor2.set(0.0);
    }

    public double getVelocityUp() {
        return motor1.getEncoder().getVelocity();
    }

    public double getVelocityDown() {
        return motor2.getEncoder().getVelocity();
    }
}
