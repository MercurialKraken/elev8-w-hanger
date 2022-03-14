// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  /** Creates a new HangerSubsystem. */
  private final WPI_TalonFX leftHanger;
  private final WPI_TalonFX rightHanger;
  private final TalonSRX piggyLeft;
  private final TalonSRX piggyRight;
  private final CANSparkMax leftNeo;
  private final CANSparkMax rightNeo;
  private final MotorControllerGroup hanger;
  private final MotorControllerGroup neo;

  public HangerSubsystem() {
    leftHanger = new WPI_TalonFX(Constants.leftHangerPort);
    rightHanger = new WPI_TalonFX(Constants.rightHangerPort);
    hanger = new MotorControllerGroup(leftHanger, rightHanger);
    piggyLeft = new TalonSRX(Constants.piggyLeftPort);
    piggyRight = new TalonSRX(Constants.piggyRightPort);
    leftNeo = new CANSparkMax(30, MotorType.kBrushless);
    rightNeo = new CANSparkMax(31, MotorType.kBrushless);
    neo = new MotorControllerGroup(leftNeo, rightNeo);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void FXMove(double pow)
  {
    rightHanger.setInverted(true);
    hanger.set(pow);
  }

  public void NeoMove(double pow)
  {
    rightNeo.setInverted(true);
    neo.set(pow);
  }


}
