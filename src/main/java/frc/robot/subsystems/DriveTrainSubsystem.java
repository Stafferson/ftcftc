package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final CANSparkMax rearLeftMotor;
  private final CANSparkMax rearRightMotor;

  private final MecanumDrive drivetrain;
  private final MecanumDriveKinematics drivetrainKinematics;
  private final MecanumDriveOdometry drivetrainOdometry;

  private final AHRS imu;

  public DriveTrainSubsystem() {
    frontLeftMotor = new CANSparkMax(
      1,
      MotorType.kBrushless
    );
    frontRightMotor = new CANSparkMax(
      2,
      MotorType.kBrushless
    );
    rearLeftMotor = new CANSparkMax(
      3,
      MotorType.kBrushless
    );
    rearRightMotor = new CANSparkMax(
      4,
      MotorType.kBrushless
    );
    
    this.frontRightMotor.setInverted(false);
    this.frontLeftMotor.setInverted(false);
    this.rearLeftMotor.setInverted(false);

    drivetrain = new MecanumDrive(
      frontLeftMotor,
      rearLeftMotor,
      frontRightMotor,
      rearRightMotor
    );

    final double wheelRadius = Units.feetToMeters(6.0) / 2.0;
    final double velocityConversionFactor = wheelRadius * Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    frontLeftMotor.getEncoder().setVelocityConversionFactor(velocityConversionFactor);
    frontRightMotor.getEncoder().setVelocityConversionFactor(velocityConversionFactor);
    rearLeftMotor.getEncoder().setVelocityConversionFactor(velocityConversionFactor);
    rearRightMotor.getEncoder().setVelocityConversionFactor(velocityConversionFactor);

    final double dist = 39.28422075083073;
    drivetrainKinematics = new MecanumDriveKinematics(
      new Translation2d(-dist, dist),
      new Translation2d(dist, dist),
      new Translation2d(-dist, -dist),
      new Translation2d(dist, -dist)
    );

    imu = new AHRS(SPI.Port.kMXP);

    drivetrainOdometry = new MecanumDriveOdometry(
      drivetrainKinematics,
      imu.getRotation2d()
    );
  }

  public void setPose(Pose2d pose) {
    drivetrainOdometry.resetPosition(
      pose,
      imu.getRotation2d()
    );
  }

  public void setPose(
    Pose2d pose,
    Rotation2d rotation
  ) {
    drivetrainOdometry.resetPosition(
      pose,
      rotation
    );
  }

  public Pose2d getPose() {
    return drivetrainOdometry.getPoseMeters();
  }

  public void drive(
    double xSpeed,
    double ySpeed,
    double rot
  ) {
    drive(
      xSpeed,
      ySpeed,
      rot,
      false
    );
  }

  public void drive(
    double xSpeed,
    double ySpeed,
    double rot,
    boolean fieldRelative
  ) {
    if (fieldRelative) {
      drivetrain.driveCartesian(
        xSpeed * 0.5,
        ySpeed * 0.5,
        rot * 0.5,
        -imu.getAngle()
      );
    } else {
      drivetrain.driveCartesian(
        xSpeed * 0.5,
        ySpeed * 0.5,
        rot * 0.5
      );
    }
  }

  public void turn(double turn) {
    drivetrain.drivePolar(
      0.0,
      0.0,
      turn
    );
  }

  public double getAngularVelocity() {
    return imu.getRate();
  }

  public MecanumDrive getDrivetrain() {
    return drivetrain;
  }

  public MecanumDriveKinematics getDrivetrainKinematics() {
    return drivetrainKinematics;
  }

  public CANEncoder getFrontLeftEncoder() {
    return frontLeftMotor.getEncoder();
  }

  public CANEncoder getFrontRightEncoder() {
    return frontRightMotor.getEncoder();
  }

  public CANEncoder getRearLeftEncoder() {
    return rearLeftMotor.getEncoder();
  }

  public CANEncoder getRearRightEncoder() {
    return rearRightMotor.getEncoder();
  }

  @Override
  public void periodic() {
    final MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds(
      getFrontLeftEncoder().getVelocity(),
      getFrontRightEncoder().getVelocity(),
      getRearLeftEncoder().getVelocity(),
      getRearRightEncoder().getVelocity()
    );

    drivetrainOdometry.update(
      imu.getRotation2d(),
      speeds
    );

    super.periodic();
  }
}