package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class BackCoralToSensor extends Command {
  private Manipulator m_manipulator;

  public BackCoralToSensor(Manipulator manipulator) {
    this.m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_manipulator.reverseIntakeMotorSlowly();
  }

  @Override
  public void end(boolean interrupted) {
  m_manipulator.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return (m_manipulator.isCoralAtBackSensor() == 1 || m_manipulator.isCoralAtBackSensor() == -1);
  }
}
