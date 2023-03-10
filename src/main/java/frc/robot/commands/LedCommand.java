// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LedSubsystem;

public class LedCommand extends CommandBase {
  
  public LedSubsystem m_LedSubsystem;
  public int actionType;
  public boolean boolToSet;
  public ShuffleboardTab tab = Shuffleboard.getTab("Leds");
  

  /** Creates a new LedCommand. */
  public LedCommand(LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_LedSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.SetAllianceColor();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
