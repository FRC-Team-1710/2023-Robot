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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class IntakeWithVision extends CommandBase {
  /** Creates a new IntakeWithVision. */
  private final IntakeSubsystem m_IntakeSubsystem;
  private final Swerve m_SwerveSub;
  private final VisionSubsystem m_VisionSubsystem;
  private final PIDController xPidController, yPidController;
  public final Timer timer = new Timer();

  public IntakeWithVision(IntakeSubsystem subsystem, Swerve subsystem1, VisionSubsystem subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = subsystem;
    m_SwerveSub = subsystem1;
    m_VisionSubsystem = subsystem2;

    addRequirements(subsystem, subsystem1, subsystem2);
    xPidController = new PIDController(.1, .05, 0);
    yPidController = new PIDController(.3, .1, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // int bestPipe = m_VisionSubsystem.pipelinePanic("Sclera");
    m_VisionSubsystem.setScleraPipeline(1);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xDisplacement = m_VisionSubsystem.getXOffsetGP();
    double yDisplacement = m_VisionSubsystem.getYOffsetGP();
    double vx = xPidController.calculate(xDisplacement, -1);
    double vy = yPidController.calculate(yDisplacement, 1);
    if (yDisplacement <= 2) {
      timer.start();
      double time = timer.get();
      SmartDashboard.putNumber("INTAKE TIMER", time);

      m_IntakeSubsystem.spin(.5);
      m_SwerveSub.drive(new Translation2d(.7, 0), vx, false, false);

    } else {
      if (m_VisionSubsystem.scleraHasTarget()) {
        m_SwerveSub.drive(new Translation2d(-vy, 0), vx, false, false);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.spin(0);
    m_SwerveSub.drive(new Translation2d(0, 0), 0, false, false);
    timer.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double time = timer.get();
    return (time > 1);

  }
}
