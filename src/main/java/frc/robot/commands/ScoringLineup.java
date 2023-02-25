// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class ScoringLineup extends CommandBase {
  /** Creates a new ScoringLineup. */
  public final VisionSubsystem m_Vision;
  public final Swerve m_Swerve;
  private String where;
  private double tagX, tagY;
  private int sideC;

  public ScoringLineup(VisionSubsystem subsystem, Swerve subsystem2, String Where) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Vision = subsystem;
    m_Swerve = subsystem2;
    where = Where;

    addRequirements(subsystem, subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double IrisX = m_Vision.getXDisToTag();
    double IrisY = m_Vision.getYDisToTag();
    int tagID = m_Vision.getTagID();

    if (tagID > 0) {
      if (tagID == 1) {
        tagX = 15.514;
        tagY = 1.072;
        sideC = -1;
      }
      if (tagID == 2) {
        tagX = 15.514;
        tagY = 2.748;
        sideC = -1;
      }
      if (tagID == 3) {
        tagX = 15.514;
        tagY = 4.424;
        sideC = -1;
      }
      if (tagID == 4) {
        tagX = 16.179;
        tagY = 6.75;
        sideC = -1;
      }
      if (tagID == 5) {
        tagX = .362;
        tagY = 6.75;
        sideC = 1;
      }
      if (tagID == 6) {
        tagX = 1.027;
        tagY = 4.424;
        sideC = 1;
      }
      if (tagID == 7) {
        tagX = 1.027;
        tagY = 2.748;
        sideC = 1;
      }
      if (tagID == 8) {
        tagX = 1.027;
        tagY = 1.072;
        sideC = 1;
      }
    }

    double currentX = tagX + (sideC * IrisX); 
    double currentY = tagY + IrisY;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
