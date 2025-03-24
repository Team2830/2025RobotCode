// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BackUp extends Command {

  private CommandSwerveDrivetrain drive;
  private Timer timer;
  private double duration;

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use velocity closed-loop control for drive motors
  /** Creates a new BackUp. */
  public BackUp(CommandSwerveDrivetrain drive, double time) {
    this.drive = drive;
    this.timer = new Timer();
    this.duration = time;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    timer.start();
    System.out.println("BackUp Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double appliedX = -0.2;

    drive.setControl(() ->
        driveRobotCentric.withVelocityX(appliedX)
            .withVelocityY(0)
            .withRotationalRate(0)
      );
    System.out.println("BackUp Executed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setControl(() ->
        driveRobotCentric.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
      );
    System.out.println("BackUp Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("BackUp Is finished " + timer.get());
    return timer.hasElapsed(duration);
  }
}
