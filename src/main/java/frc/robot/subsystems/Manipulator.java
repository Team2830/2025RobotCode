package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Manipulator extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(Constants.Manipulator.shooterMotorID, MotorType.kBrushless);

    private LaserCan frontSensor = new LaserCan(Constants.Manipulator.frontLaserId);
    private LaserCan backSensor = new LaserCan(Constants.Manipulator.backLaserId);


    private double algaeDistanceThreshold = 120;

    public Manipulator() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        try {
            frontSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            frontSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 16, 16));
            frontSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

            backSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            backSensor.setRegionOfInterest(new RegionOfInterest(0,0,8,16));
            backSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("Front Sensor", isCoralAtFrontSensor());
        SmartDashboard.putNumber("Back Sensor", isCoralAtBackSensor());
        SmartDashboard.putData(this);

        /**
         *  Front   Back
         *  2       2    = 8
         *  2       1    = 7
         *  2       0    = 6
         *  1       2    = 5
         *  1       1    = 4
         *  1       0    = 3
         *  0       2    = 2
         *  0       1    = 1
         *  0       1    = 0
         */

        SmartDashboard.putNumber("Shooter State", (isCoralAtBackSensor() + 1) + (isCoralAtFrontSensor() + 1) * 3);
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
        shooterMotor.set(-0.15);
    }

    public void reverseIntakeMotorSlowly() {
        shooterMotor.setVoltage(-0.08 * 12);
    }

    // stop motor from spinning
    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    // return 1 if there is algae, 0 if there is not algae, and -1 if there is a sensor error
    public int isCoralAtFrontSensor(){
        LaserCan.Measurement measurement = frontSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            
            SmartDashboard.putNumber("Front Sensor Distance", measurement.distance_mm);

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

            SmartDashboard.putNumber("Back Sensor Distance", measurement.distance_mm);

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