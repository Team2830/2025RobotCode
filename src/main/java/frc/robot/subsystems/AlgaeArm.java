// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  
 // private SparkMax angleMotor = new SparkMax(Constants.AlgaeArm.angleMotorId, MotorType.kBrushless);
 // private SparkMax wheelMotor = new SparkMax(Constants.AlgaeArm.wheelMotorId, MotorType.kBrushless);
  private SparkClosedLoopController pid;

  /** Creates a new AlgaeArm. */
  public AlgaeArm() {
    
    double kP = 1.8;
    double kI = 0.0;
    double kD = 0.0;

    SparkMaxConfig config = new SparkMaxConfig();
    config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.absoluteEncoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1).inverted(false);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(kP,kI,kD).positionWrappingEnabled(true).positionWrappingInputRange(0, 1);
    

    
    //angleMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    //pid = angleMotor.getClosedLoopController();

  }

  public double getRotations() {
    return 0; //angleMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean armIsDown() {
    boolean isDown = false;
    if (getRotations() < 0.05) {
      isDown = true;
    }
    return isDown;
  }

  public void setRotations(double rotations) { 
    SmartDashboard.putNumber("algaearmrots", getRotations());
    pid.setReference(rotations, ControlType.kPosition);
  }

  public void setWheelSpeed(double speed) {
    //wheelMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
