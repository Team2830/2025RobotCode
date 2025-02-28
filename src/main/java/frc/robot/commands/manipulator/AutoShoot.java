package frc.robot.commands.manipulator;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class AutoShoot extends Command {
  private Manipulator m_manipulator;
  private SwerveRequest.FieldCentricFacingAngle m_drivetrainTargetAngle;

  public AutoShoot(Manipulator manipulator, SwerveRequest.FieldCentricFacingAngle drivetrainTargetAngle) {
    this.m_manipulator = manipulator;
    this.m_drivetrainTargetAngle = drivetrainTargetAngle;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(m_manipulator.isCoralAtBackSensor() == 1) {
      m_manipulator.spinIntakeMotor(0.2);
    
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
    return ((m_manipulator.isCoralAtFrontSensor() == 1) && (m_manipulator.isCoralAtBackSensor() != 1));
  }
}
