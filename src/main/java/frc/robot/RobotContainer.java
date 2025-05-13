// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LockTrapDoor;
import frc.robot.commands.ReleaseClimber;
import frc.robot.commands.ReleaseTrapDoor;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.RunClimber;
import frc.robot.commands.drive.BackToCoralStation;
import frc.robot.commands.drive.BackUp;
import frc.robot.commands.drive.ReefCenterer;
import frc.robot.commands.elevator.KeepElevatorPosition;
import frc.robot.commands.elevator.MaintainElevatorVoltage;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.elevator.SetElevatorVoltage;
import frc.robot.commands.elevator.WaitForElevator;
import frc.robot.commands.manipulator.ActivateAlgaeArm;
import frc.robot.commands.manipulator.BackCoralToSensor;
import frc.robot.commands.manipulator.DeactivateAlgaeArm;
import frc.robot.commands.manipulator.InchForwardCoral;
import frc.robot.commands.manipulator.Intake;
import frc.robot.commands.manipulator.AutoStartIntake;
import frc.robot.commands.manipulator.Shoot;
import frc.robot.commands.manipulator.ShooterReverse;
import frc.robot.commands.manipulator.SlowShoot;
import frc.robot.generated.DrivetrainConfigs;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {
    /* Joystick Initializations */
    private final CommandXboxController joystick = new CommandXboxController(0);
    private SlewRateLimiter angle_Limiter; // limiter for driver rotation
    private SlewRateLimiter x_Limiter; // limiter for Driver X speed
    private SlewRateLimiter y_Limiter; // limiter for Driver y speed
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private Trigger elevavorIsHighTrigger;  

    /* Command Drivetrain Initializations */
    public final CommandSwerveDrivetrain drivetrain = DrivetrainConfigs.createDrivetrain();

    public double MaxSpeed = DrivetrainConfigs.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                     // speed
    private final Telemetry logger = new Telemetry(MaxSpeed); // Logger for SysId - not used when SysId commented out
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

     private final ProfiledFieldCentricFacingAngle drivetrainTargetAngle =
        new ProfiledFieldCentricFacingAngle(new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate / 0.25))
            .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    /* Other subsystem Initializations */
    public final Elevator elevator = new Elevator();
    public final Manipulator manipulator = new Manipulator();
    //public final AlgaeArm algaeArm = AlgaeArm.getInstance();
    public final Climber climber = new Climber();
    private Vision m_Vision = new Vision();


    /* Autonomous Command Declaration */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        // Limiters for Driver Joystick
        angle_Limiter = new SlewRateLimiter(Constants.Controller.joystickSlewLimiter_angle);
        x_Limiter = new SlewRateLimiter(Constants.Controller.joystickSlewLimiter_xy);
        y_Limiter = new SlewRateLimiter(Constants.Controller.joystickSlewLimiter_xy);

        elevavorIsHighTrigger = new Trigger(() -> elevator.getLevel() > 2);

        // Named Commands (Needed for Path Planner Autos)
        NamedCommands.registerCommand("Shoot", new Shoot(manipulator));
        NamedCommands.registerCommand("Intake", new Intake(manipulator).andThen(new BackCoralToSensor(manipulator)));
        NamedCommands.registerCommand("BackUpCoral", new BackCoralToSensor(manipulator));
        NamedCommands.registerCommand("Slow Shoot", new SlowShoot(manipulator));
        //NamedCommands.registerCommand("Remove Algae", new ActivateAlgaeArm(algaeArm, elevator));
        NamedCommands.registerCommand("Level1", new SetElevatorLevel(elevator, Constants.Elevator.l1Height));
        NamedCommands.registerCommand("Level2", new SetElevatorLevel(elevator, Constants.Elevator.l2Height));
        NamedCommands.registerCommand("Level3", new SetElevatorLevel(elevator, Constants.Elevator.l3Height));
        NamedCommands.registerCommand("Level4", new SetElevatorLevel(elevator, Constants.Elevator.l4Height));
        NamedCommands.registerCommand("Auto Align Left", new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.LEFT));
        NamedCommands.registerCommand("Auto Align Right", new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.RIGHT));
        NamedCommands.registerCommand("Auto Align Middle", new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.MIDDLE));
        NamedCommands.registerCommand("Back to coral station right", new BackToCoralStation(drivetrain, drivetrainTargetAngle, MaxSpeed, true));
        NamedCommands.registerCommand("Back to coral station left", new BackToCoralStation(drivetrain, drivetrainTargetAngle, MaxSpeed, false));
        NamedCommands.registerCommand("Intake Move Out", new AutoStartIntake(manipulator));
        NamedCommands.registerCommand("Wait For Elevator",new WaitForElevator(elevator));
        NamedCommands.registerCommand("Back Up", new BackUp(drivetrain, 0.2));
        

        // Configure Bindings
        configureBindings();

        // Read Selected Autonomous 
        autoChooser = new SendableChooser<Command>();

        autoChooser.setDefaultOption("Do Nothing", new PrintCommand("Nothing!"));
        autoChooser.addOption("Two Piece Right", new PathPlannerAuto("Right two piece auto"));
        autoChooser.addOption("Two Piece Left", new PathPlannerAuto("Left 2 piece auto (mirrored)", true));
        autoChooser.addOption("Drive Straight Left", new PathPlannerAuto( "Drive Straight Left"));
        autoChooser.addOption("Drive Straight Right",new PathPlannerAuto("Drive Straight Right"));
        autoChooser.addOption("Drive Straight Center", new PathPlannerAuto("Drive Straight Center"));
        autoChooser.addOption("Three Piece Right", new PathPlannerAuto("Three Piece Auto"));
        autoChooser.addOption("Drive and Align Right", new PathPlannerAuto("Drive Straight Align Right"));
        autoChooser.addOption("Drive and Align Left", new PathPlannerAuto("Drive Straight Align Left"));
        autoChooser.addOption("Right RP Auto",new PathPlannerAuto("Right RP Auto"));
        autoChooser.addOption("Left RP Auto",new PathPlannerAuto("Right RP Auto", true));
        autoChooser.addOption("Back Up Test", new BackUp(drivetrain, 0.2));
        autoChooser.addOption("2 piece test", new PathPlannerAuto("Two piece auto test mode", true));
        autoChooser.addOption("Intake First right", new PathPlannerAuto("3 pc maybe"));
        autoChooser.addOption("Intake First left", new PathPlannerAuto("Left 3 pc maybe(mirrored)", true));
        autoChooser.addOption("Front right 3 piece", new PathPlannerAuto("Front right 3 piece"));
        autoChooser.addOption("Front left 3 piece", new PathPlannerAuto("Front left 3 piece(Mirrored)", true));
        autoChooser.addOption("Center Auto", new PathPlannerAuto("Center Auto"));
        

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        operatorJoystick.povLeft().onTrue(new BackToCoralStation(drivetrain, drivetrainTargetAngle, MaxSpeed, false));
        operatorJoystick.povRight().onTrue(new BackToCoralStation(drivetrain, drivetrainTargetAngle, MaxSpeed, true));
        /************************************************ Operator Controls **************************************************/

        /** 
         * Manipulator Controls
         **/
        // IMPORTANT: Driver Right Trigger Will Also Shoot, Rest Of Controls For Operator Only
        // Left Trigger   - Intake
        // Right Trigger  - Shoot
        // Left Bumper  - Put Away Algae Arm
        // Right Bumper - Activate Algae Arm
        // Start Button  - Reverse Shooter Motors 
        operatorJoystick.leftTrigger().onTrue(new Intake(manipulator).andThen(new BackCoralToSensor(manipulator)));
        operatorJoystick.rightTrigger().or(joystick.rightBumper()).onTrue(new Shoot(manipulator));
        joystick.povUp().or(joystick.povUpLeft().or(joystick.povUpRight())).onTrue(new SlowShoot(manipulator));
        operatorJoystick.povUp().or(operatorJoystick.povUpLeft()).or(operatorJoystick.povUpRight()).whileTrue(new InchForwardCoral(manipulator));
        operatorJoystick.leftBumper().and(operatorJoystick.povDown().or(operatorJoystick.povDownLeft()).or(operatorJoystick.povDownRight())).onTrue(new ReleaseTrapDoor(climber).andThen(new WaitCommand(2)).andThen(new ReleaseClimber(climber)));
        operatorJoystick.rightBumper().onTrue(new LockTrapDoor(climber).andThen(new ResetClimber(climber)));
        //operatorJoystick.rightBumper().onTrue(new ActivateAlgaeArm(algaeArm, elevator));
        //operatorJoystick.leftBumper().onTrue(new DeactivateAlgaeArm(algaeArm));
        operatorJoystick.start().whileTrue(new ShooterReverse(manipulator)); // bindings interfere with elevator SysID bindings, normally not a problem

        /** 
         * Elevator Controls
         **/
        // Default Mode (when Constants.Elevator.elevatorMode == 0, normal setpoint operation)
        // Press A - Set Elevator to L1
        // Press B - Set Elevator to L2
        // Press X - Set Elevator to L3
        // Press Y - Set Elevator to L4
        configureElevatorBindings(Constants.Elevator.elevatorMode); // See Method or Constants Def. for Info On Other Modes

        if(Constants.ClimberConstants.usingClimber) {
            /**
             * Climber Controls
             * Womp Womp
             */
            climber.setDefaultCommand(new RunClimber(climber, operatorJoystick::getRightY));
        }
        

        /**
         * Driver Controls:
         * Start: Reset Field Oriented
         * Back: Brake Mode
         * Left and Right Triggers: Reef Auto Align
         * Left Bumper: Slow Mode
         */
        /************************************************ Driver Controls **************************************************/
        joystick.start().and(new Trigger(() -> MaxSpeed > 0)).onTrue(new InstantCommand(drivetrain::resetFieldOriented, drivetrain));
        joystick.back().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.start().and(new Trigger(() -> MaxSpeed < 0)).whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /**
         * Field Centric Driving Mode                       -- Applied By Default
         **/
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)
                        .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate)));

        /** 
         * Field Oriented w/ Constant Angle Driving Mode    -- Applied while following button held
         **/
        // A - Shooter Facing Driver's Side Middle Coral Reef
        // A and X - Shooter Facing Driver's Side Left Coral Reef
        // A and B - Shooter Facing Driver's Side Right Coral Reef
        // Y - Shooter Facing Barge Side Middle Coral Reef
        // Y and X - Shooter Facing Barge Side Left Coral Reef
        // Y and B - Shooter Facing Barge Side Right Coral Reef
        // X - Hopper Facing Left Intake Station
        // Y - Hopper Facing Right Intake Station
        configureFieldOrientedWithConstantAngleBindings();

        /**
         * Auto Align Drivetrain
         */
        joystick.leftTrigger().debounce(0.05).whileTrue(new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.LEFT));
        joystick.rightTrigger().debounce(0.05).whileTrue(new ReefCenterer(drivetrain, driveRobotCentric, m_Vision, Vision.LineupDirection.RIGHT));

        /**
         * Slow Field Centric Driving Mode                  -- Applied while Right Bumper Held
         */
        joystick.leftBumper()./*or(elevavorIsHighTrigger).*/and(() -> DriverStation.isTeleopEnabled()).whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed * 0.3)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed * 0.3)
                        .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate * 0.3)
                        .withRotationalDeadband(MaxAngularRate * 0.03)));


        /**
         * Sys ID Button Mappings For Drivetrain
         **/
        // Keep commented out when not in use
        // SignalLogger.setPath("/media/sda1/ctre-logs/"); IMPORTANT: MUST BE UNCOMMENTED FOR TESTING ELEVATOR SYSID TOO!!
        // configureSysIdBindings();
    }

    /** 
     * getAutonomousCommand - Run the Path Selected from the Auto Chooser
     **/
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * configureSysIdBindings() - Configures SysID bindings for the drivetrain. Keep
     *                            commented out when not in use
     */
    public void configureSysIdBindings() {
        joystick.leftBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::stop));
        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.povUp().and(joystick.start()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // First
        joystick.povDown().and(joystick.start()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); // Second
        joystick.povLeft().and(joystick.start()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); // Third
        joystick.povRight().and(joystick.start()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); // Fourth
    }

    /**
     * configureFieldOrientedWithConstantAngleBindings() - Hold the following button
     * combinations while driving with the normal driver joysticks to drive with 
     * selected orientation:
     * 
     * A - Shooter Facing Driver's Side Middle Coral Reef
     * A and X - Shooter Facing Driver's Side Left Coral Reef
     * A and B - Shooter Facing Driver's Side Right Coral Reef
     * Y - Shooter Facing Barge Side Middle Coral Reef
     * Y and X - Shooter Facing Barge Side Left Coral Reef
     * Y and B - Shooter Facing Barge Side Right Coral Reef
     * X - Hopper Facing Left Intake Station
     * Y - Hopper Facing Right Intake Station
     * 
     **/
    private void configureFieldOrientedWithConstantAngleBindings() {
        joystick.x().and(joystick.a()).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_LeftCloseReef))
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
        joystick.a().and(joystick.x().negate().and(joystick.b().negate())).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_MiddleCloseReef))
                          .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                          .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
        joystick.a().and(joystick.b()).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_RightCloseReef))
                         .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                         .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
        joystick.y().and(joystick.x()).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_LeftFarReef))
                      .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                      .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
        joystick.y().and(joystick.x().negate().and(joystick.b().negate())).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_MiddleFarReef))
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
        
        joystick.y().and(joystick.b()).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_RightFarReef))
                       .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                       .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)

        );

        /////////////////////////////// loading station alignment /////////////////////////
        joystick.x().and(joystick.a().negate().and(joystick.y().negate())).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_LeftCoralStation))
            .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
            .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );

        joystick.b().and(joystick.a().negate().and(joystick.y().negate())).debounce(Constants.Controller.debounce).onTrue(
            drivetrain.applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.correctForAlliance(Constants.Field.angle_RightCoralStation))
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) 
            ).until(() -> Math.abs(joystick.getRightX()) > 0.1)
        );
    }

    /**
     * configureElevatorBindings() - Depending on the provided parameter, configure
     * operator joystick button bindings for the elevator:
     * 
     * 0 [ Default ] - Standard mode, set the elevator to hold at L1, L2, L3, or 
     *                 L4 by pressing A, B, X, or Y respectivly 
     *             1 - Debugging with Set Speed Mode, controls elevator through 
     *                 directly mapping the joystick to the speed of the elevator
     *                 motors
     *             2 - Debugging with set voltage mode, controls the elevator
     *                 through always sending a constant voltage. This voltage
     *                 can be changed as the robot is enabled through btn presses
     *             3 - SysID Mode, allows for the running of SysID tests, will 
     *                 hold the elevator in the last position in between tests
     */
    private void configureElevatorBindings(int elevatorMode) {
        /**
        switch (elevatorMode) {
                case 1:
                    elevator.setDefaultCommand(new ManualElevator(elevator, () -> -operatorJoystick.getLeftY()));
                    break;
                case 2:
                    elevator.setDefaultCommand(new MaintainElevatorVoltage(elevator));
                    operatorJoystick.a().onTrue(new SetElevatorVoltage(elevator, 0));
                    operatorJoystick.b().onTrue(new SetElevatorVoltage(elevator, 1));
                    operatorJoystick.x().onTrue(new SetElevatorVoltage(elevator, 2));
                    break;
                case 3:
                    elevator.setDefaultCommand(new KeepElevatorPosition(elevator));
                    operatorJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
                    operatorJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    
                    operatorJoystick.back().and(operatorJoystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
                    operatorJoystick.back().and(operatorJoystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
                    operatorJoystick.start().and(operatorJoystick.y())
                            .whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
                    operatorJoystick.start().and(operatorJoystick.x())
                            .whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
                    break;
                default:
                    // NOTE: This is what is used in competition
                    operatorJoystick.a().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l1Height));
                    operatorJoystick.b().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l2Height));
                    operatorJoystick.x().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l3Height));
                    operatorJoystick.y().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l4Height));
                    operatorJoystick.back()
                            .whileTrue(new ManualElevator(elevator, () -> (operatorJoystick.getRightY() * -0.25)));
                    break;
            }
            **/
            operatorJoystick.a().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l1Height));
            operatorJoystick.b().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l2Height));
            operatorJoystick.x().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l3Height));
            operatorJoystick.y().onTrue(new SetElevatorLevel(elevator, Constants.Elevator.l4Height));
            operatorJoystick.back()
                    .whileTrue(new ManualElevator(elevator, () -> (operatorJoystick.getRightY() * -0.25)));
        
    }
}
