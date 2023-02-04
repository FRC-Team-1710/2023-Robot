// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.PneumaticsControlModule;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.Constants;
//import static edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;

public class PneumaticSubsystem extends SubsystemBase {

  /** Creates a new PneuSubsystem. */
  //public PneumaticsControlModule e = new PneumaticsControlModule();
  public DoubleSolenoid deezSolenoid = new DoubleSolenoid(Constants.pcmPort, PneumaticsModuleType.REVPH, Constants.kPneuForwardPort, Constants.kPneuReversePort);
  public DoubleSolenoid nutsSolenoid = new DoubleSolenoid(Constants.pcmPort, PneumaticsModuleType.REVPH, Constants.kPneu2ForwardPort, Constants.kPneu2ReversePort);

  boolean bothPneuForward;

  public PneumaticSubsystem() {

    // Sets all solenoids to the same position
    deezSolenoid.set(kReverse);
    nutsSolenoid.set(kReverse);
    bothPneuForward = false;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void SetOneSolenoidReverse() {

    deezSolenoid.set(kReverse);

  }

  public void SetOneSolenoidForward() {

    deezSolenoid.set(kForward);

  }

  public void SetTwoSolenoidsForward(){

    deezSolenoid.set(kForward);
    nutsSolenoid.set(kForward);
    bothPneuForward = true;

  }

  public void SetTwoSolenoidsReverse(){
    
    deezSolenoid.set(kReverse);
    nutsSolenoid.set(kReverse);
    bothPneuForward = false;

  }

  public void ToggleOneSolenoid(){
    deezSolenoid.toggle();
  }

  public void ToggleTwoSolenoids() {

    deezSolenoid.toggle();
    nutsSolenoid.toggle();

  }

}


