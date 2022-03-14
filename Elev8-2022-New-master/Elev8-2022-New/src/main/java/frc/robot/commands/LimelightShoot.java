// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.lang.Math;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightShoot extends SequentialCommandGroup {
  /** Creates a new ShootByDistance. */
  public LimelightShoot(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {

    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d); //Make sure limelight is the name of the networktable entry
    double dist = Constants.goalHt / Math.tan(Math.toRadians(ty+Constants.limelightAngle));
    double power = dist * Constants.ShotConstant; //constant to tune
    addCommands(new GoalSeekingCommand(driveSubsystem));
    addCommands(new GoalAligningCommand(driveSubsystem, shooterSubsystem));
    //shooterSubsystem.setHood(ty*Constants.kHood);


    // idk what this is
    // shooterSubsystem.shootPID(power, 2d); //will have to change seconds depending on how long it takes to shoot
    


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands();
  }
}
