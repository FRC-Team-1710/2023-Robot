// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera Iris = new PhotonCamera("Iris");
  // PhotonCamera Retina = new PhotonCamera("Retina");
  PhotonCamera Sclera = new PhotonCamera("Sclera");

  public VisionSubsystem() {

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

    SmartDashboard.putNumber("Tag X Dis", getXDisToTag());
    SmartDashboard.putNumber("Tag Y Dis", getYDisToTag());
    SmartDashboard.putNumber("angle target", angleToTag());
    SmartDashboard.putNumber("straight dis", straightToTag());
    SmartDashboard.putNumber("hypo", Math.sqrt((getXDisToTag() * getXDisToTag()) + (getYDisToTag() * getYDisToTag())));
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

  public void setScleraPipeline(String newPipeline) {
    if (newPipeline == "cube") {
      Sclera.setPipelineIndex(0);
    }
    if (newPipeline == "cone") {
      Sclera.setPipelineIndex(1);
    }
  }

  public int pipelinePanic(String cam) {
    if (cam == "Sclera") {
      var results = Sclera.getLatestResult();
      Sclera.setPipelineIndex(0);
      double coneTar = results.getBestTarget().getArea();
        Sclera.setPipelineIndex(1);
      double cubeTar = results.getBestTarget().getArea();
      if (coneTar >= cubeTar){
        return 0;
      } else {
        return 1;
      }
      } else{
        return 0;
      }
    }
  

  public double getXDisToTag() {
    var resultsIris = Iris.getLatestResult();
    if (resultsIris.hasTargets()) {
      return (resultsIris.getBestTarget().getBestCameraToTarget().getX() * 39.37);
    } else {
      return 0;
    }
  }

  public double getYDisToTag() {
    var resultsIris = Iris.getLatestResult();
    if (resultsIris.hasTargets()) {
      return (resultsIris.getBestTarget().getBestCameraToTarget().getY() * 39.37);
    } else {
      return 0;
    }
  }

  public double angleToTag() {
    var resultsIris = Iris.getLatestResult();
    double angle;
    if (resultsIris.hasTargets()) {
      angle = resultsIris.getBestTarget().getBestCameraToTarget().getRotation().getAngle() * (180 / 3.14) - 180;
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

  public double getXOffsetGP(String pipeline) {
    if (pipeline == "cone") {
      Sclera.setPipelineIndex(0);
    } else {
      Sclera.setPipelineIndex(1);
    }
    var resultsSclera = Sclera.getLatestResult();
    double offset;
    if (resultsSclera.hasTargets()) {
      offset = resultsSclera.getBestTarget().getYaw();
    } else {
      offset = 0;
    }
    return offset;
  }
}

// micah was here