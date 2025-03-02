// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Newton;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Vision {
    public enum LineupDirection {
        LEFT,
        RIGHT
    }

    private PhotonCamera m_Camera = new PhotonCamera("Global_Shutter_Camera");
    public final Transform3d robotToCam = new Transform3d(new Translation3d(Distance.ofBaseUnits(9.237, Inches), Distance.ofBaseUnits(0, Inches), Distance.ofBaseUnits(7.851, Inches)), new Rotation3d());
    private final AprilTagFieldLayout m_Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonPoseEstimator m_GlobalPoseEstimator = new PhotonPoseEstimator(m_Field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    private PhotonPoseEstimator m_LocalPoseEstimator = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    private Pose3d m_RelevantTagLocation;
    private Pose3d m_GoalLocation;
    private final Translation2d LEFT_OFFSET = new Translation2d(-Units.inchesToMeters(19.5), -Units.inchesToMeters(6.5));
    private final Translation2d RIGHT_OFFSET = new Translation2d(-Units.inchesToMeters(19.5), Units.inchesToMeters(6.5)); 
    private final Field2d m_Field2d = new Field2d();
    private int tagToLookAt = 0;

    public Vision() {
        m_Camera.setPipelineIndex(0);
        SmartDashboard.putData("Vision Pose", m_Field2d);
        m_GlobalPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Initialize reef lineup with a visible AprilTag.
     * @param direction Lineup to the left or right.
     * @return Whether an AprilTag was seen to lineup to.
     */
    public boolean initializeReefLineup(LineupDirection direction) {

        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        // if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        //     m_LocalPoseEstimator.setLastPose(m_Field.getTagPose(7).get());
        // } else {
        //     m_LocalPoseEstimator.setLastPose(m_Field.getTagPose(18).get());
        // }

        if( ! results.isEmpty()) {
            System.out.println("Got apriltag");
            PhotonPipelineResult result = results.get(results.size() - 1);
            
            try {
                m_LocalPoseEstimator.update(result);
                tagToLookAt = result.getBestTarget().fiducialId;
                m_RelevantTagLocation = m_Field.getTagPose(tagToLookAt).get();
                if(direction == LineupDirection.LEFT) {
                    m_GoalLocation = m_RelevantTagLocation.plus(new Transform3d(new Transform2d(LEFT_OFFSET.rotateBy(m_RelevantTagLocation.getRotation().toRotation2d()), m_RelevantTagLocation.getRotation().toRotation2d().plus(Rotation2d.k180deg))));
                } else {
                    m_GoalLocation = m_RelevantTagLocation.plus(new Transform3d(new Transform2d(RIGHT_OFFSET.rotateBy(m_RelevantTagLocation.getRotation().toRotation2d()), m_RelevantTagLocation.getRotation().toRotation2d().plus(Rotation2d.k180deg))));
                }
            } catch(Exception e) {
                return false;
            }

            return true;
        } else {
            System.out.println("no apriltag");
            return false;
        }
    }

    public EstimatedRobotPose getEstimatedLocalPose() {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        if( ! results.isEmpty()) {
            Optional<EstimatedRobotPose> estimatedPose = m_LocalPoseEstimator.update(results.get(results.size() - 1));

            if(estimatedPose.isPresent() && results.get(results.size() - 1).getBestTarget().getFiducialId() == tagToLookAt) {
                System.out.println("Setting robot pose");
                m_Field2d.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
                return estimatedPose.get();
            } else {
                return null;
            }
        } else {
            return null;
        }
    }

    public Pose2d getLocalPoseError(Pose2d drivetrainPose) {
        SmartDashboard.putNumber("Vision Align Rotation Target", m_GoalLocation.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber("Vision Align Current Rotation", drivetrainPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision Align Y Target", Units.metersToInches(m_GoalLocation.getY()));
        SmartDashboard.putNumber("Vision Align Y Actual", Units.metersToInches(drivetrainPose.getY()));
        SmartDashboard.putNumber("Vision Align Y Tag", Units.metersToInches(m_RelevantTagLocation.getY()));
        return m_GoalLocation.toPose2d().relativeTo(drivetrainPose);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
        m_GlobalPoseEstimator.setReferencePose(previousEstimatedRobotPose);

        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        if( ! results.isEmpty()) {
            return m_GlobalPoseEstimator.update(results.get(results.size() - 1));
        } else {
            return null;
        }
    }

    public void debugVision() {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                SmartDashboard.putNumber("yaw left- or right+",target.getYaw());
                SmartDashboard.putNumber("pitch up- or down+",target.getPitch());
            }
        }
    }

    public double getClosestTargetYaw() {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                return target.getYaw();
            }
        }
        return Integer.MAX_VALUE;
    }
    public double getClosestTargetPitch() {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                var target = result.getBestTarget();
                return target.getPitch();
            }
        }
        return Integer.MAX_VALUE;
    }

    public double getRobotCentricRotToTarget() {
        boolean targetVisible = false;
        double targetYaw = 0.0;

        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        return (-1.0 * targetYaw);
    }

    
}
