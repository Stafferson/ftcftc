// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
public class Robot extends TimedRobot {

  //private static final String SPI = null;
  private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
  //private Spark rightFront = new Spark(15);
  private CANSparkMax leftFront = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax leftRear = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax rightRear = new CANSparkMax(1, MotorType.kBrushless);
  //private PIDController pidController = new PIDController(0.025, 0.001, 0.0015);
  //private MecanumTest drivetrain = new MecanumTest();

  private int left_x = 0;
  private int left_y = 1;
  private int right_x = 4;
  private int right_y = 5;
  private int right_trigger = 3;
  private int left_trigger = 2;

  private double k = 0.6;

  MecanumDrive myRobot = new MecanumDrive(leftRear, leftFront, rightFront, rightRear);
  Joystick gamepad1 = new Joystick(0);

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initiali2zation code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    //leftFront.setInverted(true);
    //leftRear.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    double r = Math.hypot(gamepad1.getRawAxis(left_x), gamepad1.getRawAxis(left_y));
    double robotAngle = Math.atan2(gamepad1.getRawAxis(left_y), gamepad1.getRawAxis(left_x)) - Math.PI / 4;
    double rightX = -1 * gamepad1.getRawAxis(right_x);
    final double v1 = r * Math.cos(robotAngle) + rightX;
    final double v2 = r * Math.sin(robotAngle) - rightX;
    final double v3 = r * Math.sin(robotAngle) + rightX;
    final double v4 = r * Math.cos(robotAngle) - rightX;

    leftFront.set(k * v3);
    rightFront.set(k * v2);
    leftRear.set(k * v1);
    rightRear.set(k * v4);

    /*myRobot.setSafetyEnabled(true);
    if (gamepad1.getRawButton(1)) {
      ahrs.reset();
    }
    myRobot.driveCartesian(gamepad1.getX(), gamepad1.getY(), gamepad1.getTwist(), ahrs.getAngle());
    Timer.delay(0.005);*/

    //this.pidController.reset();
    //drivetrain.drive(gamepad1.getRawAxis(right_x), gamepad1.getRawAxis(right_y), gamepad1.getRawAxis(left_x), false);

    /*leftFront.set(gamepad1.getRawAxis(left_y));
    leftRear.set(gamepad1.getRawAxis(left_x));
    rightFront.set(gamepad1.getRawAxis(right_y));
    rightRear.set(gamepad1.getRawAxis(right_x));*/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
