package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public static final class Field {
        // Rotation Targets Used By Field Oriented w/ Constant
        public static final Rotation2d angle_LeftCloseReef = Rotation2d.fromDegrees(120);
        public static final Rotation2d angle_MiddleCloseReef = Rotation2d.fromDegrees(180);
        public static final Rotation2d angle_RightCloseReef = Rotation2d.fromDegrees(-120);

        public static final Rotation2d angle_LeftFarReef = Rotation2d.fromDegrees(60);
        public static final Rotation2d angle_MiddleFarReef = Rotation2d.fromDegrees(0);
        public static final Rotation2d angle_RightFarReef = Rotation2d.fromDegrees(-60);

        public static final Rotation2d angle_LeftCoralStation = Rotation2d.fromDegrees(126);
        public static final Rotation2d angle_RightCoralStation= Rotation2d.fromDegrees(234);
    }

    public static final class Controller {
        // Joystick Slew Limiter Values
        public static final double joystickSlewLimiter_angle = 8;
        public static final double joystickSlewLimiter_xy = 4;

        public static final double debounce = 0.03;
    }

    public static final class Swerve {
        // Routine to Apply for sysID (0 - translation; 1 - steer; 2 - rotation)
        public static final int RoutineToApplySysID = 2;

        // CAN IDs
        public static final int kPidgeonID = 13;

        public static final int kFrontLeftDriveMotorId = 12;
        public static final int kFrontLeftSteerMotorId = 11;
        public static final int kFrontLeftEncoderId = 10;

        public static final int kFrontRightDriveMotorId = 9;
        public static final int kFrontRightSteerMotorId = 8;
        public static final int kFrontRightEncoderId = 7;

        public static final int kBackLeftDriveMotorId = 6;
        public static final int kBackLeftSteerMotorId = 5;
        public static final int kBackLeftEncoderId = 4;

        public static final int kBackRightDriveMotorId = 3;
        public static final int kBackRightSteerMotorId = 2;
        public static final int kBackRightEncoderId = 1;

        // Encoder Offsets (in rotations)
        public static final double kFrontLeftEncoderOffset = -0.363037109375;
        public static final double kFrontRightEncoderOffset = 0.462158203125;
        public static final double kBackLeftEncoderOffset = -0.287109375;
        public static final double kBackRightEncoderOffset = -0.04248046875;

        // Module Positions
        public static final double kFrontLeftXPos = 13.25;
        public static final double kFrontLeftYPos = 10.25;

        public static final double kFrontRightXPos = 13.25;
        public static final double kFrontRightYPos = -10.25;

        public static final double kBackLeftXPos = -13.25;
        public static final double kBackLeftYPos = 10.25;

        public static final double kBackRightXPos = -13.25;
        public static final double kBackRightYPos = -10.25;

        // Motor Inversions
        public static final boolean kFrontLeftSteerMotorInverted = true;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final boolean kFrontRightSteerMotorInverted = true;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final boolean kBackLeftSteerMotorInverted = true;
        public static final boolean kBackLeftEncoderInverted = false;

        public static final boolean kBackRightSteerMotorInverted = true;
        public static final boolean kBackRightEncoderInverted = false;

        // max currents / max torque control for drive motors (keep positive)
        public static final double peakForwardTorqueCurrent = 130;
        public static final double peakReverseTorqueCurrent = 130;
        public static final double kSlipCurrent = 120;

        // max current for steer motor
        public static final double steerStatorCurrentLimit = 60; // lower this even more?

        // theoretical free speed (m/s) at 12 V applied output, needs to be tuned
        public static final double linearVelocityAt12V = 3.92;

        // drive ratios
        public static final double kCoupleRatio = 3.5714285714285716;
        public static final double kDriveGearRatio = 8.142857142857142;
        public static final double kSteerGearRatio = 21.428571428571427;
        public static final double kWheelRadius = 2; // in inches

        // steer motor gains
        public static final double steerKP = 17; // 17
        public static final double steerKI = 0.005; // 0.005
        public static final double steerKD = 0.05; // 0.05
        public static final double steerKS = 0.18328;
        public static final double steerKV = 0.1069225;
        public static final double steerKA = 0.002041125;

        // drive motor gains
        public static final double driveKP = 0.01;
        public static final double driveKI = 0.000;
        public static final double driveKD = 0.000;
        public static final double driveKS = 0.200;
        public static final double driveKV = 0.0134;
    }

    public static final class Elevator {
        public static final int leftMotorID = 14;
        public static final int rightMotorID = 15;

        public static final int elevatorMode = 0; // 0 - Standard Mode; 1 - Debugging with Set Speed Mode
                                                  // 2 - Debugging with Set Voltage Mode 3 - SysID Mode
                                                  // defaults to standard mode

        public static final double inchesPerRot = Math.PI * 1.88900 * (1.0 / 12.0) * 2; // pi * sproket * gear ratio *
                                                                                        // doubled for pulley
        public static final double inchesMultiplier = 1.079913607; // multiply by input inches to get actual inches that
                                                                   // should be sent to motor -- TODO: account for this
                                                                   // in the inchesPerRot variable above
        public static final double elevatorkS = 0.45; // found by determining the output necessary to overcome static
                                                      // friction
        public static final double elevatorkV = 0.00; // manually tuned value - keep at zero
        public static final double elevatorkA = 0.01; // manually tuned value
        public static final double elevatorkG = 25; // found by determining the output necessary to hold at constant
                                                      // height
        public static final double elevatorkP = 10; // manually tuned value
        public static final double elevatorkI = 0.00; // No output for integrated error
        public static final double elevatorkD = 0.05; // manually tuned value

        public static final double mm_cruiseVelocity = 80; // rotations per second
        public static final double mm_acceleration = 160; // rotations per second, per second
        public static final double mm_jerk = 1600; // rotations per second, per second, per second

        public static final double sysID_rampRate = 0.5; // volts per second
        public static final double sysID_stepVoltage = 1.2; // volts

        // elevatorBaseHeight is the distance from the base of the elevator carridge to
        // the floor in inches
        public static final double elevatorBaseHeight = 0;

        // the constant value in each variable below is the height the base of the
        // carridge should go to for that level.
        // elevatorBaseHeight is subtracted to find the actual distance the elevator
        // must travel. only edit the constant !!
        public static final double l1Height = -1.0 - elevatorBaseHeight;
        public static final double l2Height = 9 - elevatorBaseHeight;
        public static final double l3Height = 27 - elevatorBaseHeight;
        public static final double l4Height = 51 - elevatorBaseHeight;
    }
    

    public static final class Manipulator {
        public static final int shooterMotorID = 16;
        public static final int algaeMotorID = 17;
        public static final int frontLaserId = 18;
        public static final int backLaserId = 19;
    }

    public static final class AlgaeArm {
        public static final int angleMotorId = 20;
        public static final int wheelMotorId = 21;
        
        public static final double kP = 0.001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class ClimberConstants {
        public static final boolean usingClimber = false;
        public static final int climberMotor = 22;
        public static final double ratchetUnlockedAngle = 0;
        public static final double ratchedLockedAngle = 10;
        public static final double frontReleasedAngle = 0;
        public static final double frontLockedAngle = 20;
        public static final double backLockedAngle = 20;
        public static final double backReleasedAngle = 0;

    }
}
