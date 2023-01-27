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

  public DigitalOutput allianceColorOutput = new DigitalOutput(0);
  public DigitalOutput conusCubumOutput = new DigitalOutput(1);
  public DigitalOutput hasOrWantsOutput = new DigitalOutput(2);
  public DigitalOutput miscOutput = new DigitalOutput(3);
  public ShuffleboardTab tab = Shuffleboard.getTab("Leds");
  public boolean redAlliance;
  public boolean cubum = false;
  public boolean has = false;
  
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    

    
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Cubus", cubum);
    SmartDashboard.putBoolean("Has", has);

  }

  public void SetAllianceColor(){
    if (DriverStation.getAlliance() == Alliance.Red){
      redAlliance = true;
    } else {
      redAlliance = false;
    }
    allianceColorOutput.set(redAlliance);
  }

  public void SetConumCubus(boolean cubum){
    conusCubumOutput.set(cubum);
    this.cubum = cubum;
  }

  public void SetWantsOrHas(boolean has){
    hasOrWantsOutput.set(has);
    this.has = has;
  }

  public void ToggleConumCubus(){
    SetConumCubus(!cubum);
  }

  public void ToggleWantsOrHas(){
    SetWantsOrHas(!has);
  }


}
