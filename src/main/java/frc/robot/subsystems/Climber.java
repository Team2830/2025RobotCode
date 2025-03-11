// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private SparkMax motor = new SparkMax(Constants.ClimberConstants.climberMotor, MotorType.kBrushless);
  private Servo ratchetServo = new Servo(2);
  private Servo frontTrapDoorServo = new Servo(1);
  private Servo backTrapDoorServo = new Servo(0);

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberToSpeed(double speed){
    motor.set(speed);

    if(speed > 0.1){
      ratchetServo.setAngle(Constants.ClimberConstants.ratchetUnlockedAngle);
    } else {
      ratchetServo.setAngle(Constants.ClimberConstants.ratchedLockedAngle);
    }
  }

  public void lockTrapDoor(){
    frontTrapDoorServo.setAngle(Constants.ClimberConstants.frontLockedAngle);
    backTrapDoorServo.setAngle(Constants.ClimberConstants.backLockedAngle);
  }

  public void releaseTrapDoor(){
    frontTrapDoorServo.setAngle(Constants.ClimberConstants.frontReleasedAngle);
    frontTrapDoorServo.setAngle(Constants.ClimberConstants.backReleasedAngle);
  }
}


