// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DisableAlgae;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.StoreAlgae;
import frc.robot.commands.drive.ReefCenterer;
import frc.robot.commands.elevator.KeepElevatorPosition;
import frc.robot.commands.elevator.MaintainElevatorLevel;
import frc.robot.commands.elevator.MaintainElevatorVoltage;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.elevator.SetElevatorVoltage;
import frc.robot.commands.manipulator.BackCoralToSensor;
import frc.robot.commands.manipulator.Intake;
import frc.robot.commands.manipulator.Shoot;
import frc.robot.commands.manipulator.ShooterReverse;
import frc.robot.generated.DrivetrainConfigs;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {
    private double MaxSpeed = DrivetrainConfigs.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

     private final ProfiledFieldCentricFacingAngle drivetrainTargetAngle =
        new ProfiledFieldCentricFacingAngle(new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate / 0.25))
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    

    private static final Rotation2d angle_LeftCloseReef = Rotation2d.fromDegrees(120);
    private static final Rotation2d angle_MiddleCloseReef = Rotation2d.fromDegrees(180);
    private static final Rotation2d angle_RightCloseReef = Rotation2d.fromDegrees(-120);

    private static final Rotation2d angle_LeftFarReef = Rotation2d.fromDegrees(60);
    private static final Rotation2d angle_MiddleFarReef = Rotation2d.fromDegrees(0);
    private static final Rotation2d angle_RightFarReef = Rotation2d.fromDegrees(-60);

    private static final Rotation2d angle_LeftCoralStation = Rotation2d.fromDegrees(150);
    private static final Rotation2d angle_RightCoralStation= Rotation2d.fromDegrees(240);



    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = DrivetrainConfigs.createDrivetrain();

    public final Elevator elevator = new Elevator();
    public final Manipulator manipulator = new Manipulator();
    
    private SlewRateLimiter angle_Limiter;
    private SlewRateLimiter x_Limiter;
    private SlewRateLimiter y_Limiter;

    private Trigger leftTrigger;
    private Trigger rightTrigger;
    private AlgaeArm algaeArm = new AlgaeArm();

    private Vision m_Vision = new Vision();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        angle_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_angle);
        x_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);
        y_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);

        leftTrigger = new Trigger(() -> operatorJoystick.getLeftTriggerAxis() > 0.2);
        rightTrigger = new Trigger(() -> operatorJoystick.getRightTriggerAxis() > 0.2);

        configureBindings();

        NamedCommands.registerCommand("Shoot", new Shoot(manipulator));
        NamedCommands.registerCommand("Intake", new Intake(manipulator));
        NamedCommands.registerCommand("BackUpCoral", new BackCoralToSensor(manipulator));
        NamedCommands.registerCommand("Remove Algae", new RemoveAlgae(algaeArm));
        NamedCommands.registerCommand("L1", new SetElevatorLevel(elevator, 1)); 
        NamedCommands.registerCommand("L2", new SetElevatorLevel(elevator, 2)); 
        NamedCommands.registerCommand("L3", new SetElevatorLevel(elevator, 3)); 
        NamedCommands.registerCommand("L4", new SetElevatorLevel(elevator, 4)); 
        NamedCommands.registerCommand("Maintain Level", new MaintainElevatorLevel(elevator));

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        
    }

    private void configureBindings() {
        SignalLogger.setPath("/media/sda1/ctre-logs/");
        //////////////////////////////////////// Manipulator Controls (Operator Joystick) //////////////////////////////////////////////
        operatorJoystick.leftBumper().onTrue(new Intake(manipulator).andThen(new BackCoralToSensor(manipulator)) );
        operatorJoystick.rightBumper().or(joystick.rightTrigger()).onTrue(new Shoot(manipulator) );
        //operatorJoystick.start().whileTrue(new ShooterReverse(manipulator));
  
        rightTrigger.whileTrue(new RemoveAlgae(algaeArm));
        leftTrigger.whileTrue(new StoreAlgae(algaeArm));
        algaeArm.setDefaultCommand(new DisableAlgae(algaeArm));
        //////////////////////////////////////// Elevator Controls (Operator Joystick) //////////////////////////////////////////////
        switch(Constants.Elevator.elevatorMode) {
            case 1: 
                // 1 - Debugging with Set Speed Mode, controls elevator through directly mapping the joystick
                //     to the speed of the elevator motors
                elevator.setDefaultCommand(new ManualElevator(elevator, () -> -operatorJoystick.getLeftY()));
                break;
            case 2:
                // 2 - Debugging with Set Voltage Mode, controls the elevator through always sending a 
                //     constant voltage. This voltage can be changed as the robot is enabled through buttons
                elevator.setDefaultCommand(new MaintainElevatorVoltage(elevator));
                operatorJoystick.a().onTrue(new SetElevatorVoltage(elevator, 0));
                operatorJoystick.b().onTrue(new SetElevatorVoltage(elevator, 1));
                operatorJoystick.x().onTrue(new SetElevatorVoltage(elevator, 2));
                break;
            case 3:
                // 3 - SysID Mode, allows for the running of SysID tests, will hold the elevator in the last position in between tests
                elevator.setDefaultCommand(new KeepElevatorPosition(elevator));
                operatorJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
                operatorJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

                operatorJoystick.back().and(operatorJoystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
                operatorJoystick.back().and(operatorJoystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
                operatorJoystick.start().and(operatorJoystick.y()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
                operatorJoystick.start().and(operatorJoystick.x()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
                break;
            default:
                // Default - Standard Mode, control elevator through buttons corresponding to L1, L2, L3, and L4, defaulting to a safe position for the elevator
                elevator.setDefaultCommand(new MaintainElevatorLevel(elevator));
                operatorJoystick.a().onTrue(new SetElevatorLevel(elevator, 1));
                operatorJoystick.b().onTrue(new SetElevatorLevel(elevator, 2));
                operatorJoystick.x().onTrue(new SetElevatorLevel(elevator, 3));
                operatorJoystick.y().onTrue(new SetElevatorLevel(elevator, 4));
                operatorJoystick.back().whileTrue(new ManualElevator(elevator, () -> (operatorJoystick.getRightY() * -0.25)));
                break;
        }
        
        ////////////////////////////////////////// Swerve Controls (Driver Joystick) ////////////////////////////////////////////////
        joystick.start().onTrue(new InstantCommand(drivetrain::resetFieldOriented, drivetrain));
        // joystick.start().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.back().whileTrue(drivetrain.applyRequest(() -> brake));                       
        
        // Default Drive Mode is Field Centric
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    )
        );

        // Slow Mode (Field Centric) While Right Bumper Held
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed * 0.3) // Drive forward with negative Y (forward)
                    .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed * 0.3) // Drive left with negative X (left)
                    .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate * 0.3) // Drive counterclockwise with negative X (left)
            ) 
        );

        // Slow Mode (Robot Centric) While Left Bumper Held
        joystick.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveRobotCentric.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed * 0.3) // Drive forward with negative Y (forward)
                    .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed * 0.3) // Drive left with negative X (left)
                    .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate * 0.3) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.povLeft().whileTrue(new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.LEFT));
        joystick.povRight().whileTrue(new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.RIGHT));

        // SysID Commands
        /**
        joystick.leftBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::stop));
        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // First
        joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); // Second
        joystick.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));     // Third
        joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));    // Fourth
        **/
        
        // Hold A/B/X/Y Combination to translate with a target rotational angle corresponding to a side of the reef
        joystick.x().and(joystick.a().debounce(0.15)).whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_LeftCloseReef)
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );
        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_MiddleCloseReef)
                          .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                          .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );
        joystick.a().and(joystick.b().debounce(0.15)).whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_RightCloseReef)
                         .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                         .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );
        joystick.y().and(joystick.x()).debounce(0.15).whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_LeftFarReef)
                      .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                      .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );
        joystick.y().whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_MiddleFarReef)
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );
        
        joystick.y().and(joystick.b().debounce(0.15)).whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_RightFarReef)
                       .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                       .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )

        );
        /////////////////////////////// loading station alignment /////////////////////////
        joystick.x().whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_LeftCoralStation)
            .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
            .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );

        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(angle_RightCoralStation)
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            )
        );

        
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
