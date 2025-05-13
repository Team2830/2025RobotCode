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

    private PhotonCamera m_LeftCamera = new PhotonCamera("Global_Shutter_Camera");
    private PhotonCamera m_RightCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    public final Transform3d robotToCamLeft = new Transform3d(new Translation3d(Units.inchesToMeters(7.6541), Units.inchesToMeters(10.415526), Units.inchesToMeters(14.625)), new Rotation3d(0, Units.degreesToRadians(5.32), Units.degreesToRadians(-19.92)));
    public final Transform3d robotToCamRight = new Transform3d(new Translation3d(Units.inchesToMeters(7.6541), Units.inchesToMeters(-10.415526), Units.inchesToMeters(14.75)), new Rotation3d(0, Units.degreesToRadians(5.32), Units.degreesToRadians(19.92)));
    private final AprilTagFieldLayout m_Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonPoseEstimator m_GlobalPoseEstimatorLeftCamera = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLeft);
    private PhotonPoseEstimator m_GlobalPoseEstimatorRightCamera = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCamRight);
    private PhotonPoseEstimator m_LocalPoseEstimatorLeftCamera = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCamLeft);
    private PhotonPoseEstimator m_LocalPoseEstimatorRightCamera = new PhotonPoseEstimator(m_Field, PoseStrategy.LOWEST_AMBIGUITY, robotToCamRight);

    private Pose3d m_RelevantTagLocation;
    private Pose3d m_GoalLocation;

    private final Translation2d LEFT_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(-6.5));
    private final Translation2d RIGHT_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(6.5)); 
    private final Translation2d MIDDLE_OFFSET = new Translation2d(Units.inchesToMeters(20.5), Units.inchesToMeters(0)); 

    private int tagToLookAt = 0;
    private boolean usingLocalPoseEstimate = false;

    private LineupDirection currentLineupDirection = LineupDirection.MIDDLE;

    public Vision() {
        m_LeftCamera.setPipelineIndex(0);
        m_RightCamera.setPipelineIndex(0);
    }

    public boolean isUsingLocalPoseEstimate() {
        return usingLocalPoseEstimate;
    }

    // TODO: This can still be optimized to eliminate situations where we look at the wrong tag
    // Can return null if there is not a suitable target
    private PhotonTrackedTarget getTarget(PhotonPipelineResult leftResult, PhotonPipelineResult rightResult, Pose2d drivetrainPose) {
        PhotonTrackedTarget bestTarget = null;
        double bestAngleDifference = 45; // Kevin's favorite number
        
        if(leftResult != null) {
            for(PhotonTrackedTarget currentTarget : leftResult.getTargets()) {
                Rotation2d currentAngleDifference = drivetrainPose.getRotation().rotateBy(Rotation2d.k180deg).minus(m_Field.getTagPose(currentTarget.getFiducialId()).get().getRotation().toRotation2d());
                double angleDifferenceDouble = currentAngleDifference.getDegrees();

                if(Math.abs(angleDifferenceDouble) < bestAngleDifference && Math.abs(angleDifferenceDouble) < 30) {
                    bestAngleDifference = Math.abs(angleDifferenceDouble);
                    bestTarget = currentTarget;
                }
            }
        }

        if(rightResult != null) {
            for(PhotonTrackedTarget currentTarget : rightResult.getTargets()) {
                Rotation2d currentAngleDifference = drivetrainPose.getRotation().rotateBy(Rotation2d.k180deg).minus(m_Field.getTagPose(currentTarget.getFiducialId()).get().getRotation().toRotation2d());
                double angleDifferenceDouble = currentAngleDifference.getDegrees();

                if(Math.abs(angleDifferenceDouble) < bestAngleDifference && Math.abs(angleDifferenceDouble) < 30) {
                    bestAngleDifference = Math.abs(angleDifferenceDouble);
                    bestTarget = currentTarget;
                }
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
        this.currentLineupDirection = direction;

        List<PhotonPipelineResult> leftResults = m_LeftCamera.getAllUnreadResults();
        List<PhotonPipelineResult> rightResults = m_RightCamera.getAllUnreadResults();
        
        PhotonPipelineResult leftResult = null;

        if(leftResults != null && ! leftResults.isEmpty()) {
            leftResult = leftResults.get(leftResults.size() - 1);
        }

        PhotonPipelineResult rightResult = null;

        if(rightResults != null && ! rightResults.isEmpty()) {
            rightResult = rightResults.get(rightResults.size() - 1);
        }
        
        if(leftResult == null && rightResult == null) {
            return false;
        }

        try {
            if(leftResult != null) {
                m_LocalPoseEstimatorLeftCamera.update(leftResult);
            }

            if(rightResult != null) {
                m_LocalPoseEstimatorRightCamera.update(rightResult);
            }
            
            PhotonTrackedTarget target = getTarget(leftResult, rightResult, drivetrainPose);

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
    }

    public EstimatedRobotPose getEstimatedLocalPose() {
        List<PhotonPipelineResult> leftResults = m_LeftCamera.getAllUnreadResults();
        List<PhotonPipelineResult> rightResults = m_RightCamera.getAllUnreadResults();

        PhotonPipelineResult leftResult = null;
        PhotonPipelineResult rightResult = null;
        Optional<EstimatedRobotPose> leftEstimatedPose = null;
        Optional<EstimatedRobotPose> rightEstimatedPose = null;

        if(leftResults != null && ! leftResults.isEmpty()) {
            leftResult = leftResults.get(leftResults.size() - 1);

            if((leftResult.getBestTarget() != null) && (leftResult.getBestTarget().getFiducialId() == tagToLookAt)) {                
                leftEstimatedPose = m_LocalPoseEstimatorLeftCamera.update(leftResult);
            }
        }

        if(rightResults != null && ! rightResults.isEmpty()) {
            rightResult = rightResults.get(rightResults.size() - 1);

            if((rightResult.getBestTarget() != null) && rightResult.getBestTarget().getFiducialId() == tagToLookAt) {                
                rightEstimatedPose = m_LocalPoseEstimatorRightCamera.update(rightResult);
            }
        }
        
        if(this.currentLineupDirection == LineupDirection.LEFT) {
            if(leftEstimatedPose != null && leftEstimatedPose.isPresent()) {
                return leftEstimatedPose.get();
            } else if(rightEstimatedPose != null && rightEstimatedPose.isPresent()) {
                //return rightEstimatedPose.get();
                return null;
            } else {
                return null;
            }
        } else {
            if(rightEstimatedPose != null && rightEstimatedPose.isPresent()) {
                return rightEstimatedPose.get();
            } else if(leftEstimatedPose != null && leftEstimatedPose.isPresent()) {
                //return leftEstimatedPose.get();
                return null;
            } else {
                return null;
            }
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
        List<PhotonPipelineResult> LeftResults = m_LeftCamera.getAllUnreadResults();
        List<PhotonPipelineResult> RightResults = m_RightCamera.getAllUnreadResults();

        
        if(LeftResults != null && ! LeftResults.isEmpty()) {
            return m_GlobalPoseEstimatorLeftCamera.update(LeftResults.get(LeftResults.size() - 1));
        // } else if(RightResults != null && ! RightResults.isEmpty()) {
        //     return m_GlobalPoseEstimatorRightCamera.update(RightResults.get(RightResults.size() - 1));
        } else {
            return null;
        }
    }

    public void ledOn() {
        // m_Camera.setLED(VisionLEDMode.kOn);
    }

    public void ledFlash() {
        // m_Camera.setLED(VisionLEDMode.kBlink);
    }

    public void ledOff() {
        // m_Camera.setLED(VisionLEDMode.kOff);
    }

    public void debugVision() {
        // List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
        // if (!results.isEmpty()) {
        //     var result = results.get(results.size() - 1);
        //     if (result.hasTargets()) {
        //         var target = result.getBestTarget();
        //         SmartDashboard.putNumber("yaw left- or right+",target.getYaw());
        //         SmartDashboard.putNumber("pitch up- or down+",target.getPitch());
        //     }
        // }
    }

    // public double getClosestTargetYaw() {
    //     List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
    //     if (!results.isEmpty()) {
    //         var result = results.get(results.size() - 1);
    //         if (result.hasTargets()) {
    //             var target = result.getBestTarget();
    //             return target.getYaw();
    //         }
    //     }
    //     return Integer.MAX_VALUE;
    // }
    // public double getClosestTargetPitch() {
    //     List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
    //     if (!results.isEmpty()) {
    //         var result = results.get(results.size() - 1);
    //         if (result.hasTargets()) {
    //             var target = result.getBestTarget();
    //             return target.getPitch();
    //         }
    //     }
    //     return Integer.MAX_VALUE;
    // }

    // public double getRobotCentricRotToTarget() {
    //     boolean targetVisible = false;
    //     double targetYaw = 0.0;

    //     List<PhotonPipelineResult> results = m_Camera.getAllUnreadResults();
    //     if (!results.isEmpty()) {
    //         var result = results.get(results.size() - 1);
    //         if (result.hasTargets()) {
    //             // At least one AprilTag was seen by the camera
    //             for (var target : result.getTargets()) {
    //                 if (target.getFiducialId() == 7) {
    //                     // Found Tag 7, record its information
    //                     targetYaw = target.getYaw();
    //                     targetVisible = true;
    //                 }
    //             }
    //         }
    //     }

    //     return (-1.0 * targetYaw);
    // }

    
}
