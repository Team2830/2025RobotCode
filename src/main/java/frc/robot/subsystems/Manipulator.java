package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class Manipulator extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(Constants.Manipulator.shooterMotorID, MotorType.kBrushless);

    private LaserCan frontSensor = new LaserCan(Constants.Manipulator.frontLaserId);
    private LaserCan backSensor = new LaserCan(Constants.Manipulator.backLaserId);


    private double algaeDistanceThreshold = 120
    ;

    public Manipulator() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        try {
            frontSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            frontSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            frontSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

            backSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            backSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            backSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("Front Sensor", isCoralAtFrontSensor());
        SmartDashboard.putNumber("Back Sensor", isCoralAtBackSensor());
    }

    // spin the motor slow for intake
    public void spinIntakeMotor(double speed) {
        shooterMotor.set(speed);
    }

    // spin the motor fast for shooting
    public void spinOuttakeMotor() {
        shooterMotor.set(0.3);
    }

    public void reverseIntakeMotor() {
        shooterMotor.set(-0.5);
    }

    public void reverseIntakeMotorSlowly() {
        shooterMotor.set(-0.12);
    }

    // stop motor from spinning
    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    // return 1 if there is algae, 0 if there is not algae, and -1 if there is a sensor error
    public int isCoralAtFrontSensor(){
        LaserCan.Measurement measurement = frontSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < algaeDistanceThreshold){
                return 1;  
            } else {
                return 0;
            }
        }
        else {
            return -1;
        }
    }

    // return 1 if there is algae, 0 if there is not algae, and -1 if there is a sensor error
    public int isCoralAtBackSensor(){
        LaserCan.Measurement measurement = backSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < algaeDistanceThreshold){
                return 1;  
            } else {
                return 0;
            }
        }
        else {
            return -1;
        }
    }

}