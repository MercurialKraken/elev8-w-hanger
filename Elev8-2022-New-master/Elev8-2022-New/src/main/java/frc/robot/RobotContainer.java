// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeServo;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MoveByDistanceCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SwerveByDist;
import frc.robot.commands.SwerveCommand;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Commands
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem,0);

  // IO Devices
  public static Joystick joy1 = new Joystick(0);

  // public static Encoder encR = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  // public static Encoder encL = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
 
  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  // public static RelativeEncoder FR_encoder;
  // public static RelativeEncoder BR_encoder;
  // public static RelativeEncoder FL_encoder; 
  // public static RelativeEncoder BL_encoder;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(driveCommand);
    //intakeSubsystem.setDefaultCommand(intakeCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton shooterButton = new JoystickButton(joy1, 11 );
    // shooterButton.whenActive(new ShooterCommand(shooterSubsystem, 0.5));
    // shooterButton.whenPressed(new ShooterCommand(shooterSubsystem, 0.7));
    // shooterButton.whenReleased(new ShooterCommand(shooterSubsystem, 0));

    JoystickButton intakeButton = new JoystickButton(joy1, Constants.intakeButtonNum);
    intakeButton.toggleWhenPressed(new IntakeCommand(intakeSubsystem, -0.5));

    JoystickButton feederServoButton = new JoystickButton(joy1, Constants.feederServoButtonNum);
    feederServoButton.whenActive(new IntakeServo(shooterSubsystem, 90));
    feederServoButton.whenReleased(new IntakeServo(shooterSubsystem, 180));

    // JoystickButton feederServoButton = new JoystickButton(joy1, Constants.FeederServoPort);
    // feederServoButton.whenActive(new IntakeServo(shooterSubsystem, 0));
    // feederServoButton.whenReleased(new IntakeServo(shooterSubsystem, 1));

    // JoystickButton autoButton = new JoystickButton(joy1, 4);
    // autoButton.whenPressed(new SwerveByDist(driveSubsystem, 5, 5));

    // JoystickButton feederButton = new JoystickButton(joy1, Constants.feederButtonNum);
    // feederButton.whenActive(new IntakeCommand(intakeSubsystem, 0, -0.5));
    // feederButton.whenReleased(new IntakeCommand(intakeSubsystem, 0, 0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    //return new SwerveByDist(driveSubsystem, 1, 1);
    //return new SwerveCommand(driveSubsystem, 90, 1);
    return new MoveByDistanceCommand(driveSubsystem, 1);
  }

  public static double getY(Joystick joy, double deadband) {
    double value = -1 * joy.getY();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

  public static double getZ(Joystick joy, double deadband) {
    double value = joy.getZ();
    if (Math.abs(value) < deadband) return 0;
    return value;
  }

}
