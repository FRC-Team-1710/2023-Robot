// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class TagLineup extends CommandBase {
  /** Creates a new TagLineup. */
  private final Swerve m_SwerveSub;
  private final VisionSubsystem m_VisionSubsystem;
  private final PIDController xPidController, yPidController, aPidController;
  public boolean done;

  public TagLineup(Swerve subsystem1, VisionSubsystem subsystem2) {

    m_SwerveSub = subsystem1;
    m_VisionSubsystem = subsystem2;

    addRequirements(subsystem1, subsystem2);
    xPidController = new PIDController(.05, .05, 0);
    yPidController = new PIDController(.05, .05, 0);
    aPidController = new PIDController(.005, .05, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_VisionSubsystem.irisHasTarget()) {
      if (!m_SwerveSub.odometryDisabled) {
          m_SwerveSub.odometryDisabled = true;
      }
    double currentX = m_VisionSubsystem.getCurrentX();
    double currentY = m_VisionSubsystem.getCurrentY();
    double currentA = m_VisionSubsystem.angleToTag();
    int tagID = m_VisionSubsystem.getTagID();

    double xsp = m_VisionSubsystem.getIDX(tagID) + (m_VisionSubsystem.getIDXSc(tagID) * 1);
    double ysp = m_VisionSubsystem.getIDY(tagID) + (m_VisionSubsystem.getIDYSc(tagID) * 0);

      double vx = xPidController.calculate(currentX, xsp);
    double vy = yPidController.calculate(currentY, ysp);
    double va = aPidController.calculate(currentA, 180);

      m_SwerveSub.drive(new Translation2d(vx, vy), va, false, false);


  } else {
    m_SwerveSub.drive(new Translation2d(0, 0), 0, false, false);
      m_SwerveSub.odometryDisabled = false;
      done = true;
  }

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
