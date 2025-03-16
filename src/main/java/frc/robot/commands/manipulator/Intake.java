package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class Intake extends Command {
  private Manipulator m_manipulator;

  public Intake(Manipulator manipulator) {
    this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  @Override
  public void initialize() {
    
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
      return m_manipulator.isCoralAtBackSensor() != 1 && m_manipulator.isCoralAtFrontSensor() == 1;
  }
}
