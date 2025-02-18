// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {

  private SparkMax angleMotor = new SparkMax(Constants.AlgaeArm.angleMotorId, MotorType.kBrushless);
  private SparkMax wheelMotor = new SparkMax(Constants.AlgaeArm.wheelMotorId, MotorType.kBrushless);


  /** Creates a new AlgaeArm. */
  public AlgaeArm() {
    SparkBaseConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.01);

    angleMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setangle(double angle) {
    angleMotor.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void setWheelSpeed(double speed) {
    wheelMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
