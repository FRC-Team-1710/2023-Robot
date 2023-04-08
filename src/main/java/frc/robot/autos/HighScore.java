// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HighScore extends SequentialCommandGroup {
    /**
     * Creates a new HighScore.
     * 
     * @param m_ArmSubsystem
     * @param m_PneumaticSubsystem
     */
    public HighScore(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem) {

        PathPlannerTrajectory trajectory0;
        Pose2d initialPose;

        if (DriverStation.getAlliance() == Alliance.Red) {
            trajectory0 = PathPlanner.loadPath("backwards red", new PathConstraints(2.5, 2.5));
            initialPose = trajectory0.getInitialPose();
        } else {
            trajectory0 = PathPlanner.loadPath("backwards Copy", new PathConstraints(2.5, 2.5));
            initialPose = trajectory0.getInitialPose();
        }

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        PPSwerveControllerCommand path0 = new PPSwerveControllerCommand(
                trajectory0,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(driveP, 0, driveD),
                new PIDController(driveP, 0, driveD),
                new PIDController(rotP, 0, rotD),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                new ArmSet2PtPath(m_ArmSubsystem,
                        135, 177, 235, 23,
                        40, 30, 80, 35,
                        .3, .1, 0, .6, .25, 0,
                        .35, .1, 0, .35, .1, 0,
                        7, 10, 2, 4),
                new WaitCommand(.25),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                new WaitCommand(.25),
                path0.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                        146, 180, 185, 178,
                        40, 13, 50, 5,
                        .2, .2, 0, .4, .2, 0,
                        .1, .2, 0, .3, .1, 0,
                        4, 4, 2, 2)));
    }
}
