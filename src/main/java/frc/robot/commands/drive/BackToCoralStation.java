package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class BackToCoralStation extends Command {
  private CommandSwerveDrivetrain m_driveTrain;
  private ProfiledFieldCentricFacingAngle m_driveTrainTargetAngle;
  private boolean m_isRight;  
  private double m_maxSpeed;

  private Timer m_Timer = new Timer();

  private static final double speed = 0.4;
 
  public BackToCoralStation(CommandSwerveDrivetrain driveTrain, ProfiledFieldCentricFacingAngle driveTrainTargetAngle, double MaxSpeed, boolean isRight) {
    this.m_driveTrain = driveTrain;
    this.m_driveTrainTargetAngle = driveTrainTargetAngle;
    this.m_maxSpeed = MaxSpeed;
    this.m_isRight = isRight;
    addRequirements(this.m_driveTrain);
  }

  @Override
  public void initialize() {
    m_Timer.restart();
    m_Timer.start();
    
  }

  @Override
  public void execute() {
    m_driveTrain.setControl(
        () -> m_driveTrainTargetAngle.withTargetDirection (
            m_isRight ? Constants.Field.angle_RightCoralStation : Constants.Field.angle_LeftCoralStation
        )
        .withVelocityX(speed * m_maxSpeed * (m_isRight ? -1 : -1))
        .withVelocityY(speed * m_maxSpeed * (m_isRight ? -1 : 1))
      );
  }

  @Override
  public void end(boolean interrupted) {

    m_driveTrain.setControl(
        () -> m_driveTrainTargetAngle.withTargetDirection (
            m_isRight ? Constants.Field.angle_RightCoralStation : Constants.Field.angle_LeftCoralStation
        )
        .withVelocityX(0)
        .withVelocityY(0));
   
  }

  // @Override
  public boolean isFinished() {
    System.out.println("Timer: " + m_Timer.get());
    return m_Timer.hasElapsed(1.0);
  }
}
