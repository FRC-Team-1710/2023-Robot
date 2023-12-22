// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmSetAngles extends CommandBase {
  /** Creates a new ArmCommand. */
  private final ArmSubsystem m_subsystem;
  private double hangle, uangle, HDF, UDF, HP,HI,HD,UP,UI,UD,HSR,USR;
  public ArmSetAngles(ArmSubsystem subsystem,
  double Hangle, double Uangle, double hdf, double udf,
  double hp, double hi, double hd, double up, double ui, double ud, double hsr, double usr) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(subsystem);
   hangle = Hangle;
   uangle = Uangle;
   HDF = hdf;
   UDF = udf;
   HP = hp;
   HI = hi;
   HD = hd;
   UP = up;
   UI = ui;
   UD = ud;
   HSR = hsr;
   USR = usr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setAngles(hangle, uangle, HDF, UDF, HP,HI,HD,UP,UI,UD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(hangle - m_subsystem.getHPos()) < HSR && Math.abs(uangle - m_subsystem.getUPos()) < USR);
  }
}