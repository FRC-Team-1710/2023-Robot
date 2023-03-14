// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class moveArmWithTheSticks extends CommandBase {
  /** Creates a new manualArm. */
  private final ArmSubsystem m_subsystem;
  private DoubleSupplier hspeed, uspeed;
  private BooleanSupplier SILLY;
  private double shspeed, suspeed;
  
  public moveArmWithTheSticks(ArmSubsystem subsystem, DoubleSupplier Hspeed, DoubleSupplier Uspeed, BooleanSupplier silly) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
   hspeed = Hspeed;
   uspeed = Uspeed;
   SILLY = silly;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(hspeed.getAsDouble()) > .1){
      shspeed = hspeed.getAsDouble();
    }else {
      shspeed = 0;
    }
    if(Math.abs(uspeed.getAsDouble()) > .1){
      suspeed = uspeed.getAsDouble();
    }else {
      suspeed = 0;
    }
    if(SILLY.getAsBoolean()){
      m_subsystem.manualMove(-shspeed, suspeed);
    } else{
      m_subsystem.manualMove(0, 0);
    }
     
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
