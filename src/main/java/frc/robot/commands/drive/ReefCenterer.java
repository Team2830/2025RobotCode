package frc.robot.commands.drive;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefCenterer extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.RobotCentric m_driveRobotCentric;
private Vision m_Vision;
private Vision.LineupDirection m_Direction;
  private boolean shouldTryLineup = false;
  private boolean isLinedUp = false;
  
  public ReefCenterer (CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRobotCentric,Vision vision, Vision.LineupDirection direction) {
    this.m_drivetrain = drivetrain;
    this.m_driveRobotCentric = driveRobotCentric;
    this.m_Vision = vision;
    this.m_Direction = direction;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    shouldTryLineup = m_Vision.initializeReefLineup(m_Direction);
  }

  @Override
  public void execute() {
    EstimatedRobotPose visionResult = m_Vision.getEstimatedLocalPose();

    if(visionResult != null) {
      m_drivetrain.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), visionResult.timestampSeconds);
    }

    Pose2d error = m_Vision.getLocalPoseError(m_drivetrain.getState().Pose);

    double x = 0.0;
    double y = 0.0;
    double rotation = 0.0;

    if(Math.abs(error.getRotation().getDegrees()) > 0.5) {
      rotation = error.getRotation().getRadians() * 1.0;
    } else if(Math.abs(Units.metersToInches(error.getY())) > 0.5) {
      y = error.getY() * 1.0;
    }else if (Math.abs(Units.metersToInches(error.getX())) >0.5) {
      x = error.getX() * 1.0;
    } else {
      isLinedUp = true;
    }

    final double appliedX = x;
    final double appliedY = y;
    final double appiedRotation = rotation;

    m_drivetrain.applyRequest(() ->
        m_driveRobotCentric.withVelocityX(appliedX)
            .withVelocityY(appliedY)
            .withRotationalRate(appiedRotation)
    );
    

}

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return (! shouldTryLineup) || isLinedUp;
  }
}
