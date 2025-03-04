package frc.robot.commands.drive;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefCenterer extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
  // private SwerveRequest.RobotCentric m_driveRobotCentric;
private Vision m_Vision;
private Vision.LineupDirection m_Direction;
  private boolean shouldTryLineup = false;
  private boolean isLinedUp = false;
  private ProfiledPIDController m_RotationController = new ProfiledPIDController(1.0, 0, 0, new Constraints(1.0, 1.0));
  SwerveDrivePoseEstimator m_PoseEstimator;
  private boolean hasAddedVision = false;

  private boolean foundRotation = false;
  private boolean foundY = false;
  private boolean foundX = false;

      private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors


  public ReefCenterer (CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRobotCentric,Vision vision, Vision.LineupDirection direction) {
    this.m_drivetrain = drivetrain;
    // this.m_driveRobotCentric = driveRobotCentric;
    this.m_Vision = vision;
    this.m_Direction = direction;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    shouldTryLineup = m_Vision.initializeReefLineup(m_Direction);
    m_Vision.ledOn();
    
    System.out.println("Trying lineup");
  }

  @Override
  public void execute() {
    if (! shouldTryLineup) {
      return;
    }

    EstimatedRobotPose visionResult = m_Vision.getEstimatedLocalPose();

    // if(visionResult != null && m_PoseEstimator == null) {
    //   m_PoseEstimator = new SwerveDrivePoseEstimator(m_drivetrain.getKinematics(), 
    //     m_drivetrain.getRotation3d().toRotation2d(), 
    //     m_drivetrain.getState().ModulePositions, 
    //     new Pose2d(visionResult.estimatedPose.getTranslation().toTranslation2d(), 
    //     m_drivetrain.getRotation3d().toRotation2d()));
    // }

    // if(m_PoseEstimator == null) return;

    // m_PoseEstimator.update(m_drivetrain.getRotation3d().toRotation2d(), m_drivetrain.getState().ModulePositions);

    if(visionResult != null) {
      SmartDashboard.putNumber("Y Before Vision", m_drivetrain.getState().Pose.getY());

      Matrix<N3, N1> visionStdDev = new Matrix<>(Nat.N3(), Nat.N1());
      visionStdDev.set(0, 0, Units.inchesToMeters(1.0)); // X StdDev
      visionStdDev.set(1, 0, Units.inchesToMeters(1.0)); // Y StdDev
      visionStdDev.set(2, 0, Units.degreesToRadians(20)); // Heading StdDev
      // m_drivetrain.resetPose(new Pose2d(visionResult.estimatedPose.getTranslation().toTranslation2d(), m_drivetrain.getRotation3d().toRotation2d()));

      // m_PoseEstimator.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), visionResult.timestampSeconds, visionStdDev);

      // m_drivetrain.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(visionResult.timestampSeconds), visionStdDev);
      m_drivetrain.addVisionMeasurement(visionResult.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(visionResult.timestampSeconds));

      SmartDashboard.putNumber("Y After Vision", m_drivetrain.getState().Pose.getY());
      System.out.println("Adding vision measurement");
    }

    Pose2d error = m_Vision.getLocalPoseError(m_drivetrain.getState().Pose);

    double x = 0.0;
    double y = 0.0;
    double rotation = 0.0;

    if(Math.abs(error.getRotation().getDegrees()) > 3.0) {
      System.out.println("Error: " + error.getRotation().getDegrees());
      SmartDashboard.putNumber("Rotation Error", error.getRotation().getDegrees());
      rotation = -1.0 * m_RotationController.calculate(error.getRotation().getRadians());
      // rotation = 1.2 * error.getRotation().getRadians();
      System.out.println("Fixing rotation, speed: " + rotation);

    }  else {
      foundRotation = true;
    }
    
    if(foundRotation && Math.abs(Units.metersToInches(error.getY())) > 1.0) {
      
      y = error.getY() * -3.2;
      SmartDashboard.putNumber("Y Error", Units.metersToInches(error.getY()));
      System.out.println("Fixing x");

    } else {
      foundY = true;
    }
    
    if (foundRotation && foundY && Math.abs(Units.metersToInches(error.getX())) > 1.0) {
      
      x = error.getX() * -3.2;
      System.out.println("Fixing Y");
    } else {
      foundX = true;
      isLinedUp = true;
      System.out.println("All good");
    }

    final double appliedX = x;
    final double appliedY = y;
    final double appiedRotation = rotation;

    m_drivetrain.setControl(() ->
        driveRobotCentric.withVelocityX(appliedX)
            .withVelocityY(appliedY)
            .withRotationalRate(appiedRotation)
    );
    

}

  @Override
  public void end(boolean interrupted) {
    if(isLinedUp) {
      m_Vision.ledOff();
    } else {
      m_Vision.ledFlash();
    }

    foundRotation = false;
    foundY = false;
    foundX = false;
    m_PoseEstimator = null;
    m_Vision.clear();
    shouldTryLineup = false;
    isLinedUp = false;
  }

  @Override
  public boolean isFinished() {
    System.out.println("shouldTryLineup: " + shouldTryLineup + " isLinedUp: " + isLinedUp);
    return (! shouldTryLineup) || isLinedUp;
  }
}
