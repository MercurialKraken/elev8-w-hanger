// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveByDist extends SequentialCommandGroup {
  double x,y;
  DriveSubsystem driveSubsystem;
  /** Creates a new SwerveByDist. */
  public SwerveByDist(DriveSubsystem driveSubsystem, double x, double y) {
    this.x = x;
    this.y = y;
    this.driveSubsystem = driveSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double dist = Math.PI * Math.sqrt(2) * Math.pow(x, 2) * Math.pow(y, 2);
    addCommands(new SwerveCommand(driveSubsystem, Math.toDegrees(Math.atan(y/x))*2, dist));
  }
}
