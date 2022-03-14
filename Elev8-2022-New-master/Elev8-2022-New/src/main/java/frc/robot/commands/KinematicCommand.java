// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class KinematicCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  double x,y,speed;
  ChassisSpeeds chassisSpeeds;
  DifferentialDriveWheelSpeeds wheelSpeeds;
  /** Creates a new KinematicCommand. */
  public KinematicCommand(DriveSubsystem driveSubsystem, double x, double y, double speed) {
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navx.reset();
    //y/x*Constants.realMaxSpeed*speed
    chassisSpeeds = new ChassisSpeeds(Constants.realMaxSpeed*speed, 0, Math.atan(y/x)/Constants.Tperiod);
    wheelSpeeds = driveSubsystem.kinematics.toWheelSpeeds(chassisSpeeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(wheelSpeeds.leftMetersPerSecond/Constants.realMaxSpeed, wheelSpeeds.rightMetersPerSecond/Constants.realMaxSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.toDegrees(Math.atan(y/x)) <= RobotContainer.navx.getAngle();
  }
}
