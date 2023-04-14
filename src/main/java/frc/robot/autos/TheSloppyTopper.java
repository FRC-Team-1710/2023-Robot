package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TheSloppyTopper extends SequentialCommandGroup {
    public TheSloppyTopper(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem, VisionSubsystem m_VisionSubsystem) {

        PathPlannerTrajectory trajectory1;
        PathPlannerTrajectory trajectory2;

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        Pose2d initialPose;

        if (DriverStation.getAlliance() == Alliance.Red){
                trajectory1 = PathPlanner.loadPath("ST Red", new PathConstraints(1.5, 1.5));
                trajectory2 = PathPlanner.loadPath("ST2 Red", new PathConstraints(2.5, 2.5));
                initialPose = new Pose2d(14.73, .5, new Rotation2d(0));
        } else {
                trajectory1 = PathPlanner.loadPath("ST Copy", new PathConstraints(1.5, 1.5));
                trajectory2 = PathPlanner.loadPath("ST2 Copy", new PathConstraints(2.5, 2.5));
                initialPose = trajectory1.getInitialPose();
        }

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new IntakeSpin(m_IntakeSubsystem, 0.5));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

        HashMap<String, Command> eventMap2 = new HashMap<>();
        eventMap2.put("Arm Up", new ArmSet2PtPath(m_ArmSubsystem,
        137, 116, 245, -30,
        30, 15, 60, 25,
        .3, .1, 0, .6, .25, 0,
        .35, .1, 0, .35, .1, 0,
        9, 10, 2.5, 4.5));

        PPSwerveControllerCommand path1 = new PPSwerveControllerCommand(
                trajectory1,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(driveP, 0, driveD),
                new PIDController(driveP, 0, driveD),
                new PIDController(rotP, 0, rotD),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);

        PPSwerveControllerCommand path2 = new PPSwerveControllerCommand(
                trajectory2,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(driveP, 0, driveD),
                new PIDController(driveP, 0, driveD),
                new PIDController(rotP, 0, rotD),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);

        FollowPathWithEvents command = new FollowPathWithEvents(
                path1,
                trajectory1.getMarkers(),
                eventMap);
        FollowPathWithEvents command2 = new FollowPathWithEvents(
                path2,
                trajectory2.getMarkers(),
                eventMap2);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsForward()),
                new ArmSet2PtPath(m_ArmSubsystem,
                140, 115, 237, -34,
                30, 15, 60, 25,
                .3, .1, 0, .6, .25, 0,
                .35, .1, 0, .35, .1, 0,
                9, 10, 2.5, 4.5).raceWith(new WaitCommand(5.5)),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                new WaitCommand(.15),
                command.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                140, 123, 182, 116,
                30, 15, 20, 7,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 2, 2)),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                command2,
                new WaitCommand(.25),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
               /*  new ArmSet2PtPath(m_ArmSubsystem,
                        155, 233, 185, 227,
                        40, 13, 100, 10,
                        .2, .2, 0, .4, .2, 0,
                        .1, .2, 0, .3, .1, 0,
                        4, 4, 2, 2),*/
                        new IntakeSpin(m_IntakeSubsystem, -0.5),
                        new WaitCommand(2),
                        new IntakeSpin(m_IntakeSubsystem, 0));

    }
}