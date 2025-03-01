package frc.robot.commands.drive;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class ReefCenterer extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.RobotCentric m_driveRobotCentric;
private Vision m_Vision;
  public ReefCenterer (CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRobotCentric,Vision vision) {
    this.m_drivetrain = drivetrain;
    this.m_driveRobotCentric = driveRobotCentric;
    this.m_Vision = vision;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double yaw = m_Vision.getClosestTargetYaw();
    double pitch = m_Vision.getClosestTargetPitch();

    double ySpeed;
    double xSpeed;

    /**
    if(yaw>0){//update yaw value
        ySpeed=
    }

    m_drivetrain.applyRequest(() ->
        m_driveRobotCentric.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(null)
    );
    **/ 
    

}

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
