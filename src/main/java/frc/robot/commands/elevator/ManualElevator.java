package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
  private Elevator m_elevator;
  private DoubleSupplier m_speed;
  /** Creates a new ManualElevator. 
   * 
   * Should be set as default command when it is desired to control the elevators speed with a joystick.
  */
  public ManualElevator(Elevator elevator, DoubleSupplier speed) {
    this.m_elevator = elevator;
    this.m_speed = speed;
    addRequirements(this.m_elevator);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double finalSpeed = m_speed.getAsDouble();

    if (Math.abs(finalSpeed) < 0.1) {
      finalSpeed = 0;
      
    }

    m_elevator.setManualSpeed(m_speed.getAsDouble());
    SmartDashboard.putNumber("manual elevator speed", m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
