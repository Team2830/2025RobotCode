package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MaintainElevatorVoltage extends Command {
  private Elevator m_elevator;
  /** Creates a new MaintainElevatorVoltage
   * 
   * Should be set as default command when using elevator in set voltage mode for getting
   * feedforward values. The elevator has a variable voltage that the motors output voltages
   * will be set to when this command is run. This variable can be edited with the SetElevatorVoltage
   * command
   */
  public MaintainElevatorVoltage(Elevator elevator) {
    this.m_elevator = elevator;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_elevator.setVoltage();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
