package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TeleOpCommand extends CommandBase {
  final DriveTrainSubsystem drivetrain;
  final ShooterSubsystem shooter;
  final TurretSubsystem turret;
  final IntakeSubsystem intake;
  final IndexerSubsystem indexer;
  final PIDController pidController;
  final JoystickSubsystem joystick;
  final NetworkTableEntry tx;

  public TeleOpCommand(DriveTrainSubsystem drivetrain, IndexerSubsystem indexer, ShooterSubsystem shooter, TurretSubsystem turret, IntakeSubsystem intake, JoystickSubsystem joystick) {
    this.drivetrain = drivetrain;
    this.indexer = indexer;
    this.joystick = joystick;
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.pidController = new PIDController(0.025, 0.001, 0.0015);
    this.tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

    addRequirements(drivetrain, joystick, shooter, turret, intake);
  }

  double prevError = 0.0;
  double shooterAcceptableVelocity = 4550.0;

  @Override
  public void execute() {
    var leftStick = joystick.getLeftStick();
    var rightStick = joystick.getRightStick();
    var isShooting = joystick.getRB2() > 0.1;

    if (isShooting) {
      var error = this.tx.getDouble(0.0);
      var value = this.pidController.calculate(error, 0.0 - 0.015 * drivetrain.getAngularVelocity());

      var turretValue = 0.05 * error;

      //turret.spin(-turretValue);
      drivetrain.drive(rightStick.x, rightStick.y, -value + leftStick.x, false);

      prevError = error;

      /* if(shooter.getVelocity() > shooterAcceptableVelocity) {
        this.indexer.spin();
      } */
    } else {
      this.pidController.reset();
      drivetrain.drive(rightStick.x, rightStick.y, leftStick.x, false);
    }

    if (joystick.isXPressed()) {
      shooter.spit();
    } else {
      shooter.spin(Math.max(joystick.getRB2(), 0.2));
    }
    
    // System.out.println(shooter.getVelocity());
    if(joystick.getRB1().get()) {
      this.indexer.spin();
    }
    else {
      this.indexer.stop();
    }

    if (joystick.isDpadLeft()) {
      turret.spin(-1);
    } else if (joystick.isDpadRight()) {
      turret.spin(1);
    } else if (!isShooting) {
      turret.spin(0);
    }

    if (joystick.getLB1().get()) {
      intake.spin(1.0);
    } else if (joystick.getLB2() > 0.5) {
      intake.spin(-1.0);
    } else {
      intake.stop();
    }
  }
}
