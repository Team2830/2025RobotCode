package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MaintainElevatorLevel extends Command {
  private Elevator m_elevator;
  /** Creates a new ElevatorManual. 
   * 
   * Should be set as default command when it is desired to keep the elevator at the height
   * of the last target level. Currently used for the standard mode, but if we dont get 
   * another encoder it might be smart to get rid of this and have the elevator go to its
   * very bottom position and reset its level when not in use. This way the relative encoder
   * won't get thrown off by hitting the cages/chains under the barge.
  */
  public MaintainElevatorLevel(Elevator elevator) {
    this.m_elevator = elevator;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_elevator.setLevel();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
