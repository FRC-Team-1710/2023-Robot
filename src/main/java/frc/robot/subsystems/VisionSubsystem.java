// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.wpilibj.DriverStation;
 import java.io.IOException;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    /** Creates a new VisionSubsystem. */
    PhotonCamera Iris = new PhotonCamera("Iris");
    // PhotonCamera Retina = new PhotonCamera("Retina");
    PhotonCamera Sclera = new PhotonCamera("Sclera");

    private PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem() {

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, Iris, Constants.robotToCam);
          
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var resultsIris = Iris.getLatestResult();
        // var resultsRetina = Retina.getLatestResult();
        var resultsSclera = Sclera.getLatestResult();

        SmartDashboard.putBoolean("Iris Target", irisHasTarget());
        // SmartDashboard.putBoolean("Retina Target", retinaHasTarget());
        SmartDashboard.putBoolean("Sclera Target", scleraHasTarget());

        SmartDashboard.putNumber("straight dis", straightToTag());
        SmartDashboard.putNumber("current X", getCurrentX());
        SmartDashboard.putNumber("current Y", getCurrentY());
        SmartDashboard.putNumber("THA X", getXDisToTag() * 39.7);
        SmartDashboard.putNumber("THA Y", getYDisToTag() * 39.7);
        SmartDashboard.putNumber("THA Y TWO", getXDisToTag() * Math.tan(angleToTag()));
        SmartDashboard.putNumber("angle ", angleToTag());

    }

    public boolean irisHasTarget() {
        var resultsIris = Iris.getLatestResult();
        if (resultsIris.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }
    /*
     * public boolean retinaHasTarget() {
     * var resultsRetina = Retina.getLatestResult();
     * if (resultsRetina.hasTargets()) {
     * return true;
     * } else {
     * return false;
     * }
     * }
     */

    public boolean scleraHasTarget() {
        var resultsSclera = Sclera.getLatestResult();
        if (resultsSclera.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }

    /*
     * public void setRetinaPipeline(String newPipeline) {
     * if (newPipeline == "cube") {
     * Retina.setPipelineIndex(0);
     * }
     * if (newPipeline == "cone") {
     * Retina.setPipelineIndex(1);
     * }
     * }
     */

    public void setScleraPipeline(int pipeline) {
        Sclera.setPipelineIndex(pipeline);
    }

    public int pipelinePanic(String cam) {
        if (cam == "Sclera") {
            var results = Sclera.getLatestResult();
            Sclera.setPipelineIndex(0);
            double coneTar = results.getBestTarget().getArea();
            Sclera.setPipelineIndex(1);
            double cubeTar = results.getBestTarget().getArea();
            if (coneTar >= cubeTar) {
                return 0;
            } else {
                return 1;
            }
        } else {
            return 0;
        }
    }

    public double getXDisToTag() {
        var resultsIris = Iris.getLatestResult();
        if (resultsIris.hasTargets()) {
            // return ((68.1 * resultsIris.getBestTarget().getPitch()) + 439);
            return (resultsIris.getBestTarget().getBestCameraToTarget().getX());
        } else {
            return 0;
        }
    }

    public double getYDisToTag() {
        var resultsIris = Iris.getLatestResult();
        if (resultsIris.hasTargets()) {
            return (resultsIris.getBestTarget().getBestCameraToTarget().getY());
        } else {
            return 0;
        }
    }

    public double angleToTag() {
        var resultsIris = Iris.getLatestResult();
        double angle;
       
        if (resultsIris.hasTargets()) {
            double rawAngle = resultsIris.getBestTarget().getBestCameraToTarget().getRotation().getZ() * (180 / 3.14);
            if (rawAngle > 0) {
                angle = 180 - rawAngle;
            }
            if (rawAngle < 0) {
                angle = 180 + rawAngle;
            } else {
                angle = 0;
            }
        } else {
            angle = 0;
        }
        return angle;
    }

    public double straightToTag() {
        var resultsIris = Iris.getLatestResult();
        double angle;
        if (resultsIris.hasTargets()) {
            angle = getXDisToTag() * Math.cos(angleToTag());
        } else {
            angle = 0;
        }
        return angle;
    }

    public double getXOffsetGP() {

        var resultsSclera = Sclera.getLatestResult();
        double offset;
        if (resultsSclera.hasTargets()) {
            offset = resultsSclera.getBestTarget().getYaw();
        } else {
            offset = 0;
        }
        return offset;
    }

    public double getYOffsetGP() {

        var resultsSclera = Sclera.getLatestResult();
        double offset;
        if (resultsSclera.hasTargets()) {
            offset = resultsSclera.getBestTarget().getPitch();
        } else {
            offset = 0;
        }
        return offset;
    }

    public int getTagID() {
        var rIris = Iris.getLatestResult();
        int tagID;
        if (rIris.hasTargets()) {
            tagID = rIris.getBestTarget().getFiducialId();
        } else {
            tagID = 0;
        }
        return tagID;
    }

    public double getIDX(int ID) {

        int tagID = ID;
        double tagX = 0;

        if (tagID == 1) {
            tagX = 15.514;
        }
        if (tagID == 2) {
            tagX = 15.514;
        }
        if (tagID == 3) {
            tagX = 15.514;
        }
        if (tagID == 4) {
            tagX = 16.179;
        }
        if (tagID == 5) {
            tagX = .362;
        }
        if (tagID == 6) {
            tagX = 1.027;
        }
        if (tagID == 7) {
            tagX = 1.027;
        }
        if (tagID == 8) {
            tagX = 1.027;
        }
        return tagX;

    }

    public int getIDXSc(int ID) {

        int tagID = ID;
        int sideC = 1;

        if (tagID == 1) {
            sideC = -1;
        }
        if (tagID == 2) {
            sideC = -1;
        }
        if (tagID == 3) {
            sideC = -1;
        }
        if (tagID == 4) {
            sideC = -1;
        }
        if (tagID == 5) {
            sideC = 1;
        }
        if (tagID == 6) {
            sideC = 1;
        }
        if (tagID == 7) {
            sideC = 1;
        }
        if (tagID == 8) {
            sideC = 1;
        }
        return sideC;
    }

    public double getCurrentX() {
        double IrisX = getXDisToTag();
        int tagID = getTagID();
        double tagX = getIDX(tagID), currentX;
        int sideC = getIDXSc(tagID);

        currentX = tagX + (sideC * IrisX);
        return currentX;
    }

    public double getIDY(int ID) {
        int tagID = ID;
        double tagY = 0;

        if (tagID == 1) {
            tagY = 1.072;
        }
        if (tagID == 2) {
            tagY = 2.748;
        }
        if (tagID == 3) {
            tagY = 4.424;
        }
        if (tagID == 4) {
            tagY = 6.75;
        }
        if (tagID == 5) {
            tagY = 6.75;
        }
        if (tagID == 6) {
            tagY = 4.424;
        }
        if (tagID == 7) {
            tagY = 2.748;
        }
        if (tagID == 8) {
            tagY = 1.072;
        }
        return tagY;
    }

    public int getIDYSc(int ID) {
        int tagID = getTagID();
        int sideC = 1;

        if (tagID == 1) {
            sideC = -1;
        }
        if (tagID == 2) {
            sideC = -1;
        }
        if (tagID == 3) {
            sideC = -1;
        }
        if (tagID == 4) {
            sideC = -1;
        }
        if (tagID == 5) {
            sideC = 1;
        }
        if (tagID == 6) {
            sideC = 1;
        }
        if (tagID == 7) {
            sideC = 1;
        }
        if (tagID == 8) {
            sideC = 1;
        }
        return sideC;
    }

    public double getCurrentY() {
        double IrisY = getYDisToTag();
        int tagID = getTagID();
        double tagY = getIDY(tagID), currentY;
        int sideC = getIDYSc(tagID), sideT = 1;

        currentY = tagY - (sideC * sideT * IrisY);

        return currentY;

    }


    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}

// micah was here