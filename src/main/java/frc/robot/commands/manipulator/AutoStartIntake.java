package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class AutoStartIntake extends Command {
  private Manipulator m_manipulator;

  public AutoStartIntake(Manipulator manipulator) {
    this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_manipulator.spinIntakeMotor(.4);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return m_manipulator.isCoralAtBackSensor() == 1;
  }
}
