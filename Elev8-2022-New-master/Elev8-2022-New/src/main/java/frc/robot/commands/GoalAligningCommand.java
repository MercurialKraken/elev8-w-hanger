// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GoalAligningCommand extends CommandBase {

  /** Creates a new GoalSeekingCommand. */
  DriveSubsystem driveSubsystem;
  ShooterSubsystem shooterSubsystem;
  double error, tx;
  //boolean tv;

  public GoalAligningCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navx.reset();
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this. tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //this.ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //this.tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
    
    this.error = tx - (RobotContainer.navx.getYaw() * 1.1);
    double correction = this.error * Constants.kPTurn;
    this.driveSubsystem.moveByAngle(correction);
  
    //this.dist = Constants.goalHt / Math.tan(ty);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterSubsystem.shoot(dist * Constants.kShoot);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(this.error) <= Math.max(1.00d, (this.tx * Constants.deadband)));
  }
}
