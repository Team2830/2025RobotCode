// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Vision {
    public enum LineupDirection {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private PhotonCamera m_Camera = new PhotonCamera("Global_Shutter_Camera");
    public final Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(9.237), Units.inchesToMeters(0), Units.inchesToMeters(7.851)), new Rotation3d());
    private final AprilTagFieldLayout m_Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonPoseEstimator m_GlobalPoseEstimator = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    private PhotonPoseEstimator m_LocalPoseEstimator = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    private Pose3d m_RelevantTagLocation;
    private Pose3d m_GoalLocation;

    private final Translation2d LEFT_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(-6.5));
    private final Translation2d RIGHT_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(6.5)); 
    private final Translation2d MIDDLE_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(0)); 

    private int tagToLookAt = 0;
    private boolean usingLocalPoseEstimate = false;

    public Vision() {
        m_Camera.setPipelineIndex(0);
    }

    public boolean isUsingLocalPoseEstimate() {
        return usingLocalPoseEstimate;
    }

    // TODO: This can still be optimized to eliminate situations where we look at the wrong tag
    // Can return null if there is not a suitable target
    private PhotonTrackedTarget getTarget(PhotonPipelineResult result, Pose2d drivetrainPose) {
        PhotonTrackedTarget bestTarget = null;
        double bestAngleDifference = 45; // Kevin's favorite number
        
        for(PhotonTrackedTarget currentTarget : result.getTargets()) {
            Rotation2d currentAngleDifference = drivetrainPose.getRotation().rotateBy(Rotation2d.k180deg).minus(m_Field.getTagPose(currentTarget.getFiducialId()).get().getRotation().toRotation2d());
            double angleDifferenceDouble = currentAngleDifference.getDegrees();

            if(Math.abs(angleDifferenceDouble) < bestAngleDifference && Math.abs(angleDifferenceDouble) < 30) {
                bestAngleDifference = Math.abs(angleDifferenceDouble);
                bestTarget = currentTarget;
            }
        }

        return bestTarget;
    }

    public void clear() {
        tagToLookAt = 0;
        usingLocalPoseEstimate = false;
    }

    /**
     * Initialize reef lineup with a visible AprilTag.
     * @param direction Lineup to the left or right.
     * @return Whether an AprilTag was seen to lineup to.
     */
    public boolean initializeReefLineup(LineupDirection direction, Pose2d drivetrainPose) {

        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        if( ! results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            
            try {
                m_LocalPoseEstimator.update(result);
                PhotonTrackedTarget target = getTarget(result, drivetrainPose);
                if(target != null) {
                    tagToLookAt = target.getFiducialId();
                } else {
                    return false;
                }
                
                m_RelevantTagLocation = m_Field.getTagPose(tagToLookAt).get();
                
                SmartDashboard.putNumber("Initial Tag Pose", m_RelevantTagLocation.getRotation().toRotation2d().getDegrees());
                SmartDashboard.putNumber("Rotated Rotation", Rotation2d.fromDegrees(m_RelevantTagLocation.getRotation().toRotation2d().getDegrees() + 180).getDegrees());
                
                if(direction == LineupDirection.LEFT) {
                    m_GoalLocation = m_RelevantTagLocation.plus(new Transform3d(new Transform2d(LEFT_OFFSET, Rotation2d.k180deg)));
                } else if (direction == LineupDirection.RIGHT) {
                    m_GoalLocation = m_RelevantTagLocation.plus(new Transform3d(new Transform2d(RIGHT_OFFSET, Rotation2d.k180deg)));
                } else {
                    m_GoalLocation = m_RelevantTagLocation.plus(new Transform3d(new Transform2d(MIDDLE_OFFSET, Rotation2d.k180deg)));
                }

                usingLocalPoseEstimate = true;
            } catch(Exception e) {
                return false;
            }

            return true;
        } else {
            return false;
        }
    }

    public EstimatedRobotPose getEstimatedLocalPose() {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        if( ! results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);

            if( ! result.hasTargets()) {
                return null;
            }

            if(result.getBestTarget().getFiducialId() == tagToLookAt) {                
                Optional<EstimatedRobotPose> estimatedPose = m_LocalPoseEstimator.update(result);
            
                if(estimatedPose.isPresent()) {
                    return estimatedPose.get();
                } else {
                    return null;
                }
            } else {
                return null;
            }
        } else {
            return null;
        }
    }

    /**
     * Returns the error pose for reef alignment.
     * @param drivetrainPose The current pose of the drivetrain.
     * @return A pose object giving information on the movement the drivetrain needs to make. Positive Y is to the right, positive X is backwards.
     */
    public Pose2d getLocalPoseError(Pose2d drivetrainPose) {
        SmartDashboard.putNumber("Vision Align Rotation Target", m_GoalLocation.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber("Vision Align Current Rotation", drivetrainPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision Align Y Target", Units.metersToInches(m_GoalLocation.getY()));
        SmartDashboard.putNumber("Vision Align Y Actual", Units.metersToInches(drivetrainPose.getY()));
        SmartDashboard.putNumber("Vision Align Y Tag", Units.metersToInches(m_RelevantTagLocation.getY()));
        Pose2d translationErrorPose = drivetrainPose.relativeTo(m_GoalLocation.toPose2d());
        Pose2d errorPose = new Pose2d(translationErrorPose.getTranslation(), m_GoalLocation.getRotation().toRotation2d().minus(drivetrainPose.getRotation()));
        return errorPose;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
        List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        
        if( ! results.isEmpty()) {
            return m_GlobalPoseEstimator.update(results.get(results.size() - 1));
        } else {
            return null;
        }
    }

    public void ledOn() {
        m_Camera.setLED(VisionLEDMode.kOn);
    }

    public void ledFlash() {
        m_Camera.setLED(VisionLEDMode.kBlink);
    }

    public void ledOff() {
        m_Camera.setLED(VisionLEDMode.kOff);
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
