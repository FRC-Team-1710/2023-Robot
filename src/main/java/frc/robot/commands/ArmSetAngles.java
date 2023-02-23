// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmSetAngles extends CommandBase {
  /** Creates a new ArmCommand. */
  private final ArmSubsystem m_subsystem;
  private double hangle, uangle, HDF, UDF;
  public ArmSetAngles(ArmSubsystem subsystem, double Hangle, double Uangle, double hdf, double udf) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
   hangle = Hangle;
   uangle = Uangle;
   HDF = hdf;
   UDF = udf;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setAngles(hangle, uangle, HDF, UDF);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(hangle - m_subsystem.getHPos()) < 3 && Math.abs(uangle - m_subsystem.getUPos()) < 3);
  }
}