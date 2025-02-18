package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivetrainConfigs {
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(Constants.Swerve.steerKP).withKI(Constants.Swerve.steerKI).withKD(Constants.Swerve.steerKD)
        .withKS(Constants.Swerve.steerKS).withKV(Constants.Swerve.steerKV).withKA(Constants.Swerve.steerKA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(Constants.Swerve.driveKP).withKI(Constants.Swerve.driveKI).withKD(Constants.Swerve.driveKD)
        .withKS(Constants.Swerve.driveKS).withKV(Constants.Swerve.driveKV);

    // The closed-loop output type to use for the steer & drive motors - affects PID/FF gains for motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the steer & drive motors
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(Constants.Swerve.kSlipCurrent);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withTorqueCurrent(new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(Amps.of(Constants.Swerve.peakForwardTorqueCurrent))
            .withPeakReverseTorqueCurrent(Amps.of(Constants.Swerve.peakReverseTorqueCurrent).times(-1))
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(Constants.Swerve.steerStatorCurrentLimit))
                .withStatorCurrentLimitEnable(true) 
        );

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("*", "./logs/example.hoot");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(Constants.Swerve.linearVelocityAt12V);
    private static final double kCoupleRatio = Constants.Swerve.kCoupleRatio;
    private static final double kDriveGearRatio = Constants.Swerve.kDriveGearRatio;
    private static final double kSteerGearRatio = Constants.Swerve.kSteerGearRatio;
    private static final Distance kWheelRadius = Inches.of(Constants.Swerve.kWheelRadius);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage kSteerFrictionVoltage = Volts.of(Constants.Swerve.steerKS);
    private static final Voltage kDriveFrictionVoltage = Volts.of(Constants.Swerve.driveKS);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(Constants.Swerve.kPidgeonID)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(Constants.Swerve.kFrontLeftEncoderOffset);
    private static final Distance kFrontLeftXPos = Inches.of(Constants.Swerve.kFrontLeftXPos);
    private static final Distance kFrontLeftYPos = Inches.of(Constants.Swerve.kFrontLeftYPos);

    // Front Right
    private static final Angle kFrontRightEncoderOffset = Rotations.of(Constants.Swerve.kFrontRightEncoderOffset);
    private static final Distance kFrontRightXPos = Inches.of(Constants.Swerve.kFrontRightXPos);
    private static final Distance kFrontRightYPos = Inches.of(Constants.Swerve.kFrontRightYPos);

    // Back Left
    private static final Angle kBackLeftEncoderOffset = Rotations.of(Constants.Swerve.kBackLeftEncoderOffset);
    private static final Distance kBackLeftXPos = Inches.of(Constants.Swerve.kBackLeftXPos);
    private static final Distance kBackLeftYPos = Inches.of(Constants.Swerve.kBackLeftYPos);

    // Back Right
    private static final Angle kBackRightEncoderOffset = Rotations.of(Constants.Swerve.kBackRightEncoderOffset);
    private static final Distance kBackRightXPos = Inches.of(Constants.Swerve.kBackRightXPos);
    private static final Distance kBackRightYPos = Inches.of(Constants.Swerve.kBackRightYPos);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            Constants.Swerve.kFrontLeftSteerMotorId, Constants.Swerve.kFrontLeftDriveMotorId, Constants.Swerve.kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, Constants.Swerve.kFrontLeftSteerMotorInverted, Constants.Swerve.kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            Constants.Swerve.kFrontRightSteerMotorId, Constants.Swerve.kFrontRightDriveMotorId, Constants.Swerve.kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, Constants.Swerve.kFrontRightSteerMotorInverted, Constants.Swerve.kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            Constants.Swerve.kBackLeftSteerMotorId, Constants.Swerve.kBackLeftDriveMotorId, Constants.Swerve.kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, Constants.Swerve.kBackLeftSteerMotorInverted, Constants.Swerve.kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            Constants.Swerve.kBackRightSteerMotorId, Constants.Swerve.kBackRightDriveMotorId, Constants.Swerve.kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, Constants.Swerve.kBackRightSteerMotorInverted, Constants.Swerve.kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
