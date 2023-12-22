// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class stopTheArmTweakin extends InstantCommand {
  /** Creates a new manualArm. */
  private final ArmSubsystem m_subsystem;

  public stopTheArmTweakin(ArmSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.manualMove(0, 0);
  }
}
