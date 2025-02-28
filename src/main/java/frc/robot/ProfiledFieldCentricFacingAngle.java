package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ProfiledFieldCentricFacingAngle implements SwerveRequest {

    public double VelocityX = 0;
    public double VelocityY = 0;

    /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();

    public double Deadband = 0;
    public double RotationalDeadband = 0;
    public Translation2d CenterOfRotation = new Translation2d();
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    public boolean DesaturateWheelSpeeds = true;
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;
    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();
    public final PhoenixPIDController HeadingController = m_fieldCentricFacingAngle.HeadingController;

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    public ProfiledFieldCentricFacingAngle(TrapezoidProfile.Constraints constraints) {
        profile = new TrapezoidProfile(constraints);
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        goal.position = TargetDirection.getRadians();  // number of radians to rotate to

        {
            var currentAngle = parameters.currentPose.getRotation(); // current number of radians rotated

            /* From ProfiledPIDController::calculate */

            // Get error which is the smallest distance between goal and measurement
            double distanceNeeded = MathUtil.angleModulus(goal.position - currentAngle.getRadians());
            
            goal.position = distanceNeeded  + currentAngle.getRadians();
            setpoint.position = currentAngle.getRadians();

            SmartDashboard.putNumber("Angle needed to move",(distanceNeeded*180/3.14));
            SmartDashboard.putNumber("Current Angle the Thinks Robots At: ", currentAngle.getDegrees());
            SmartDashboard.putNumber("Angle We Want to Be At", (goal.position*180/3.14));
        }   

        setpoint = profile.calculate(parameters.updatePeriod, setpoint, goal);
        return m_fieldCentricFacingAngle
            .withVelocityX(VelocityX)
            .withVelocityY(VelocityY)
            .withTargetDirection(Rotation2d.fromRadians(setpoint.position))
            .withTargetRateFeedforward(setpoint.velocity)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withCenterOfRotation(CenterOfRotation)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
            .withForwardPerspective(ForwardPerspective)
            .apply(parameters, modulesToApply);
    }

    /**
     * Resets the profile used for the target direction.
     *
     * @param currentHeading The current heading of the robot
     */
    public void resetProfile(Rotation2d currentHeading) {
        setpoint.position = currentHeading.getRadians();
    }


    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withVelocityX(double newVelocityX) {
        this.VelocityX = newVelocityX;
        return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     * <p>
     * The velocity in the X direction, in m/s. X is defined as forward according to
     * WPILib convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withVelocityX(LinearVelocity newVelocityX) {
        this.VelocityX = newVelocityX.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withVelocityY(double newVelocityY) {
        this.VelocityY = newVelocityY;
        return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     * <p>
     * The velocity in the Y direction, in m/s. Y is defined as to the left
     * according to WPILib convention, so this determines how fast to travel to the
     * left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withVelocityY(LinearVelocity newVelocityY) {
        this.VelocityY = newVelocityY.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the TargetDirection parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withTargetDirection(Rotation2d newTargetDirection) {
        this.TargetDirection = newTargetDirection;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public ProfiledFieldCentricFacingAngle withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
        this.ForwardPerspective = newForwardPerspective;
        return this;
    }
}
