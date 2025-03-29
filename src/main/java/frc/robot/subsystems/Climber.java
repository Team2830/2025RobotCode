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
  private Servo climberServo = new Servo(1);
  private Servo trapDoorServo = new Servo(0);

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberToSpeed(double speed){
    if (Math.abs(speed) < 0.3) {
      speed = 0;
    }
    // positive speeds climb
    // neg speeds let climber back out
    // hit latch when speed negative
    motor.set(speed);

    if(speed < 0){
      ratchetServo.setAngle(49);
    } else {
      ratchetServo.setAngle(90);
    }
  }

  public void lockTrapDoor(){
    trapDoorServo.setAngle(Constants.ClimberConstants.lockedAngle);
  }

  public void releaseTrapDoor(){
    trapDoorServo.setAngle(Constants.ClimberConstants.unlockedBackAngle);
  }

  public void releaseClimber(){
    climberServo.setAngle(90);
  }
  public void resetClimber(){
    climberServo.setAngle(0);
  }
}


