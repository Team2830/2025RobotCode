package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorLevel extends Command {
  private Elevator m_elevator;
  private double m_level;

  /** Creates a new SetElevatorLevel. 
   * 
   * Updates the variable in the elevator subsystem that determines the 
   * level the elevator will target (1-4 for levels 1 - 4). May be desired
   * to add another 0th position to bottom out the elevator for resetting 
   * the relative encoder.
  */
  public SetElevatorLevel(Elevator elevator, double level) {
    this.m_elevator = elevator;
    this.m_level = level;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setLevel(m_level);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  // @Override
  public boolean isFinished() {
    return true;
  }
}
