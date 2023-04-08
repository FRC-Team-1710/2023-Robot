// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public DigitalOutput allianceColorOutput = new DigitalOutput(2);
  public DigitalOutput gamePieceColorOutput = new DigitalOutput(3);
  public DigitalOutput blinkOutput = new DigitalOutput(4);
  public DigitalOutput visionLockedOutput = new DigitalOutput(5);
  public DigitalOutput testOutput = new DigitalOutput(6);
  public ShuffleboardTab tab = Shuffleboard.getTab("Leds");
  public boolean redAlliance;
  public boolean isPurple = false;
  public boolean isBlinking = false;

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putBoolean("isPurple", isPurple);
    //SmartDashboard.putBoolean("isSolid", isBlinking);

    if(SmartDashboard.getBoolean("ENCODER FAILURE", false)){
      SetConeOrCubeColor(true);
    }else{
      SetConeOrCubeColor(false);
    }
  }

  public void SetAllianceColor() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      redAlliance = true;
    } else {
      redAlliance = false;
    }
    allianceColorOutput.set(redAlliance);
  }

  public void SetConeOrCubeColor(boolean isPurple) {
    gamePieceColorOutput.set(isPurple);
    this.isPurple = isPurple;
  }

  public void SetBlink(boolean isBlinking) {
    blinkOutput.set(isBlinking);
    this.isBlinking = isBlinking;
  }

  public void ToggleConeOrCubeColor() {
    SetConeOrCubeColor(!isPurple);
  }

  public void ToggleBlink() {
    SetBlink(!isBlinking);
  }

  public void SetVisionPattern(boolean visionLocked){
    visionLockedOutput.set(visionLocked);
  }

  public void SetTestPattern(boolean runTest) {
    testOutput.set(runTest); 
  }

}