package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class Intake extends Command {
  private Manipulator m_manipulator;
  private int m_LastBackSensorState;
  private Timer timer;

  public Intake(Manipulator manipulator) {
    this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    m_LastBackSensorState = 0;
  }

  @Override
  public void execute() {
    if(m_manipulator.isCoralAtBackSensor() == 1) {
      m_manipulator.spinIntakeMotor(0.1);
    
    }else{
      m_manipulator.spinIntakeMotor(.4);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
  m_manipulator.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    if(m_manipulator.isCoralAtFrontSensor() == 1) {
      if( ! timer.isRunning()) {
        timer.start();
      }
    }

    SmartDashboard.putNumber("Timer", timer.get());

    // if(timer.get() > 0.07) {
    //   return true;
    // }

    // if(m_LastBackSensorState == 1) {
    //   System.out.println("Back Sensor: " + m_manipulator.isCoralAtBackSensor() + " Front Sensor: " + m_manipulator.isCoralAtFrontSensor());
    // }

    // if(m_manipulator.isCoralAtBackSensor() != 1 && m_LastBackSensorState == 1) {
    //   return true;
    // } else {
    //   m_LastBackSensorState = m_manipulator.isCoralAtBackSensor();
      
      if(m_manipulator.isCoralAtBackSensor() != 1 && m_manipulator.isCoralAtFrontSensor() == 1) {
        return true;
      }
      
      return false;
    // }


    // return ((m_manipulator.isCoralAtFrontSensor() == 1) && (m_manipulator.isCoralAtBackSensor() != 1));
  }
}
