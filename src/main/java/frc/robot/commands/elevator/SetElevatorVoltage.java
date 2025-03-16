package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorVoltage extends Command {
  private Elevator m_elevator;
  private double m_voltage;
  
  /** 
   * Creates a new ElevatorManual. 
   * Should be called to adjust the voltage being sent to the motors when MaintainElevatorVoltage is
   * being used as a default command. This is done when the elevator is in mode 2 - debugging with 
   * set voltage mode, as defined in Constants.Elevator.
  */
  public SetElevatorVoltage(Elevator elevator, double voltage) {
    this.m_elevator = elevator;
    this.m_voltage = voltage;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setVoltage(m_voltage);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
