package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class KeepElevatorPosition extends Command {
  private Elevator m_elevator;
  /** Creates a new KeepElevatorPosition 
   * 
   *  Should be set as default command when it is desired to maintain the elevator at the last
   *  recorded position once no other command is running. Used for SysID test to maintain height
   *  between tests.
  */
  public KeepElevatorPosition(Elevator elevator) {
    this.m_elevator = elevator;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_elevator.maintainElevatorPosition();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
