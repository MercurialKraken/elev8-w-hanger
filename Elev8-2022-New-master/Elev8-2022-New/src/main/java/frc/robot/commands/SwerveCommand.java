// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class SwerveCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double desiredAngle, desiredDistance, angleError, distanceError;

  /** Creates a new SwerveCommand. */
  public SwerveCommand(DriveSubsystem driveSubsystem, double desiredAngle, double desiredDistance) {
    this.driveSubsystem = driveSubsystem;
    this.desiredAngle = desiredAngle;
    this.desiredDistance = desiredDistance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navx.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.angleError = this.desiredAngle - (RobotContainer.navx.getYaw() * Constants.navxScale);
    // this.distanceError = this.desiredDistance - ((this.driveSubsystem.getAverageDistance()) * Constants.encoderScale);
    // this.driveSubsystem.swerve(this.angleError, this.distanceError);

    this.distanceError = this.desiredDistance - ((this.driveSubsystem.getAverageDistance()) * Constants.encoderScale);
    double distanceCorrection = this.distanceError * Constants.kPDist;
    // this.driveSubsystem.moveByDistance(distanceCorrection);

    this.angleError = this.desiredAngle - (RobotContainer.navx.getYaw() * 1.1);
    double angleCorrection = this.angleError * Constants.kPTurn;
    // this.driveSubsystem.moveByAngle(angleCorrection);

    this.driveSubsystem.swerve(angleCorrection, distanceCorrection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(this.angleError) <= Math.max(1.00d, (this.desiredAngle * 0.03))) && (Math.abs(this.distanceError) <= Math.max(1.00d, (this.desiredDistance * Constants.deadband)));
  }
}