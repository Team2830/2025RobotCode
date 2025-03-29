// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseVision = true;

  private final Vision m_Vision = new Vision();
  private Field2d m_field = new Field2d();

  public Robot() {
    CanBridge.runTCP();
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Auto Command Finished", m_robotContainer.getAutonomousCommand().isFinished());
    m_Vision.debugVision();
    if(kUseVision) {
      Optional<Pose2d> lastPose = m_robotContainer.drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

      if(lastPose != null && lastPose.isPresent()) {
        Optional<EstimatedRobotPose> estimatedPose = m_Vision.getEstimatedGlobalPose(lastPose.get());

        if(estimatedPose != null && estimatedPose.isPresent() && ! m_Vision.isUsingLocalPoseEstimate()) {
          m_robotContainer.drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(estimatedPose.get().timestampSeconds));
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                if (m_robotContainer.MaxSpeed > 0) {
                m_robotContainer.MaxSpeed = m_robotContainer.MaxSpeed * -1;
                }
            }
            else {
              if (m_robotContainer.MaxSpeed < 0) {
                m_robotContainer.MaxSpeed = m_robotContainer.MaxSpeed * -1;
              }
            }
        }
        }
        
        m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          if (alliance.get() == DriverStation.Alliance.Blue) {
              if (m_robotContainer.MaxSpeed > 0) {
              m_robotContainer.MaxSpeed = m_robotContainer.MaxSpeed * -1;
              }
          }
          else {
            if (m_robotContainer.MaxSpeed < 0) {
              m_robotContainer.MaxSpeed = m_robotContainer.MaxSpeed * -1;
            }
          }
      }
    } else {
      if (m_robotContainer.MaxSpeed < 0) {
        m_robotContainer.MaxSpeed = m_robotContainer.MaxSpeed * -1;
      }
    }
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
