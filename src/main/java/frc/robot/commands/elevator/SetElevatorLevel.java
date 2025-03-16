package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Elevator;

public class SetElevatorLevel extends Command {
  private Elevator m_elevator;
  private double m_level;
  private Timer m_Timer = new Timer();
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
    System.out.println("Elevator initialized");
    m_Timer.restart();
    m_Timer.start();
  }

  @Override
  public void execute() {
   // if (m_AlgaeArm.armIsDown()) 
   System.out.println("Elevator Executed");

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator ended");
  }

  // @Override
  public boolean isFinished() {
    // System.out.println("Elevtor isFinished returned true");
    // return true;
    System.out.println("Timer: " + m_Timer.get());
    return m_Timer.hasElapsed(0.1);
    // return false;
  }
}
