package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase; import frc.robot.Constants; import frc.robot.RobotContainer; import frc.robot.subsystems.DriveSubsystem;
public class DriveCommand extends CommandBase
{
  DriveSubsystem driveSubsystem;
  public DriveCommand(DriveSubsystem driveSubsystem) {this.driveSubsystem = driveSubsystem; addRequirements(driveSubsystem);}
  @Override public void initialize() {}
  @Override public void execute() { double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.deadband); double zaxis = RobotContainer.getZ(RobotContainer.joy1, Constants.deadband); driveSubsystem.arcadeInbuilt(yaxis, zaxis);}
  @Override public void end(boolean interrupted) {}
  @Override public boolean isFinished() {return false;}
}