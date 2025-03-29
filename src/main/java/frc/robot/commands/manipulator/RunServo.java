// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;

public class RunServo extends Command {
  public RunServo(Servo backServo, Servo frontServo, Servo winchServo) {
    this.backServo = backServo;
    this.frontServo = frontServo;
    //this.winchServo = winchServo;
    //addRequirements(this.algaeArm, this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      backServo.set(1);
      frontServo.set(1);
      //winchServo.set(0);
    } 
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //algaeArm.setangle(0);
    //algaeArm.setWheelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
**/