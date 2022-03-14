// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /*PORTS*/
    public static final int FR_port = 21;
    public static final int FL_port = 11;
    public static final int BR_port = 22; 
    public static final int BL_port = 12;
    public static final int ShooterPort = 5;
    public static final int IntakePort = 2;
    public static final int FeederPort = 1;
    public static final int ServoButtonNum = 6;

    public static final int piggyRightPort = 15;
    public static final int piggyLeftPort = 16;
    public static final int leftHangerPort = 17;
    public static final int rightHangerPort = 18;


    public static double arcadeMaxSpeed = 0.5;
    public static double maxSpeed = 0.3;
    public static double minSpeed = 0.1;
    public static double deadband = 0.2;

    // Encoders
    public static double encoderScale = 0.03;
    public static double rightScale = 0.25;
    public static double gearRatio = 7.31;

    public static final double G = 9.81;

    public static double kPTurn = 0.0085;
    public static double kPDist = 0.21;

    public static double wheel2wheelDist = 23.75;


    public static double navxScale = 1.1;

    public static double goalHt = 2.6416; //set this to the actual goal height (in metres preferably)
    public static double limelightAngle = 30d;
    public static double ShotConstant = 0.001;
    public static double kPShoot = 0.01;
    public static double kIShoot = 0;    
    public static double kDShoot = 1;
    // kP, kI, kD to be tuned
    public static double wheelDia = 0.152d;
    public static double rpm = 93.37;
    public static double realMaxSpeed = rpm/60*Math.PI*wheelDia;
    public static double Tperiod = 1d;

    
    public static final int shootButtonNum = 1;
    public static final int intakeButtonNum = 2;
    public static final int feederButtonNum = 4;
    public static final int feederServoButtonNum = 5;
    public static final int FeederServoPort = 10;
}
