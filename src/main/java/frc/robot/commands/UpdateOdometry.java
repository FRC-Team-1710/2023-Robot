// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateOdometry extends InstantCommand {

  private Swerve swerveSubsystem;
  private VisionSubsystem visionSubsystem;
  public UpdateOdometry(VisionSubsystem m_VisionSubsystem, Swerve m_SwerveSubsystem) {
    addRequirements(m_VisionSubsystem);

    swerveSubsystem = m_SwerveSubsystem;
    visionSubsystem = m_VisionSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(visionSubsystem.irisHasTarget()){
      swerveSubsystem.resetOdometry(visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).get().estimatedPose.toPose2d());
    }
  }
}
