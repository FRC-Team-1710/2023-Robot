// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HighScore extends SequentialCommandGroup {
    /** Creates a new HighScore. */
    public HighScore(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem) {
        PathPlannerTrajectory trajectory0 = PathPlanner.loadPath("backwards", new PathConstraints(2.5, 2));

        PPSwerveControllerCommand path0 = new PPSwerveControllerCommand(
                trajectory0,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, .05),
                new PIDController(Constants.AutoConstants.kPYController, 0, .05),
                new PIDController(0, 0, 0),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory0.getInitialPose())),
                new IntakeSpin(m_IntakeSubsystem, -.5),
                new WaitCommand(.75),
                new IntakeSpin(m_IntakeSubsystem, 0),
                path0);
    }
}
