// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax FR;
  private final CANSparkMax BR;

  private final MotorControllerGroup rightSide;


  private final CANSparkMax FL;
  private final CANSparkMax BL;
  private final MotorControllerGroup leftSide;

  public static RelativeEncoder FR_encoder;
  public static RelativeEncoder BR_encoder;
  public static RelativeEncoder FL_encoder; 
  public static RelativeEncoder BL_encoder;

  private final DifferentialDrive driveTrain;

  public final DifferentialDriveKinematics kinematics;
  public DifferentialDriveOdometry m_odometry;
  public DifferentialDriveOdometry m_pose;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    FR = new CANSparkMax(Constants.FR_port, MotorType.kBrushless);
    BR = new CANSparkMax(Constants.BR_port, MotorType.kBrushless);
    rightSide = new MotorControllerGroup(FR, BR);
    
    FL = new CANSparkMax(Constants.FL_port, MotorType.kBrushless);
    BL = new CANSparkMax(Constants.BL_port, MotorType.kBrushless);

    FR_encoder = FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    BR_encoder = BR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    FL_encoder = FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    BL_encoder = BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);


    
    leftSide = new MotorControllerGroup(FL, BL);
    
    driveTrain = new DifferentialDrive(leftSide, rightSide);

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.wheel2wheelDist));
    m_odometry = new DifferentialDriveOdometry(RobotContainer.navx.getRotation2d());


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FL_Encoder", FL_encoder.getPosition());
    // This method will be called once per scheduler run
    //m_pose = m_odometry.update(RobotContainer.navx.getRotation2d(), leftDistanceMeters, rightDistanceMeters)
    
  }

  public void arcadeInbuilt(double y, double x) {
    rightSide.setInverted(true);
    driveTrain.arcadeDrive(y * Constants.arcadeMaxSpeed, x * Constants.arcadeMaxSpeed);
  }

  public void drive(double l, double r) {
    FR.setInverted(true);
    BR.setInverted(true);

    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void driveRaw(double y) {
    drive(y * Constants.maxSpeed, y * Constants.maxSpeed);
  }

  public double getAverageDistance() {
    return ((FR_encoder.getPosition()+BR_encoder.getPosition() + FL_encoder.getPosition()+ BL_encoder.getPosition()/4)*Math.PI*Constants.wheelDia);
    //return((((FL.getEncoder().getPosition() + BR.getEncoder().getPosition())/2)/Constants.gearRatio) * Math.PI * Constants.wheelDia);
  }

  public void moveByDistance(double correction) {
    if (Math.abs(correction) < Constants.minSpeed) correction = Math.signum(correction) * Constants.minSpeed;
    if (Math.abs(correction) > Constants.maxSpeed) correction = Math.signum(correction) * Constants.maxSpeed;
    drive(correction, correction);
  }

  public void moveByAngle(double correction) {
    if (Math.abs(correction) < Constants.minSpeed) correction = Math.signum(correction) * Constants.minSpeed;
    if (Math.abs(correction) > Constants.maxSpeed) correction = Math.signum(correction) * Constants.maxSpeed;
    drive(correction, -correction);
  }

  public void swerve(double angleCorrection, double distanceCorrection) {
    double correctionLeft = distanceCorrection + angleCorrection;
    double correctionRight =  distanceCorrection - angleCorrection;

    if (Math.abs(correctionLeft) < Constants.minSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.minSpeed;
    if (Math.abs(correctionLeft) > Constants.maxSpeed) correctionLeft = Math.signum(correctionLeft) * Constants.maxSpeed;
    if (Math.abs(correctionRight) < Constants.minSpeed) correctionRight = Math.signum(correctionRight) * Constants.minSpeed;
    if (Math.abs(correctionRight) > Constants.maxSpeed) correctionRight = Math.signum(correctionRight) * Constants.maxSpeed;

    drive(correctionLeft, correctionRight);
  }

}
