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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TheSloppyTopper extends SequentialCommandGroup {
        public TheSloppyTopper(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
                        PneumaticSubsystem m_PneumaticSubsystem, VisionSubsystem m_VisionSubsystem) {

                PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("ST", new PathConstraints(1.5, 1.5));
                PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("ST2", new PathConstraints(2, 2));

                double rotP = 1.25;
                double rotD = 0.06;
                double driveP = 1.5;
                double driveD = 0.02;

                HashMap<String, Command> eventMap = new HashMap<>();
                eventMap.put("Intake", new IntakeSpin(m_IntakeSubsystem, 0.5));
                eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

                HashMap<String, Command> eventMap2 = new HashMap<>();
                eventMap2.put("Arm Up", new ArmSet2PtPath(m_ArmSubsystem,
                                143.7, 225, 238.5, 63,
                                40, 30, 80, 35,
                                .3, .1, 0, .6, .25, 0,
                                .35, .1, 0, .35, .1, 0,
                                7, 10, 2, 4));

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
                                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
                                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                                new ArmSet2PtPath(m_ArmSubsystem,
                                                143.7, 225, 238.5, 63,
                                                40, 30, 80, 35,
                                                .3, .1, 0, .6, .25, 0,
                                                .35, .1, 0, .35, .1, 0,
                                                7, 10, 2, 4),
                                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                                new WaitCommand(.15),
                                new TheSloppyTopper3(m_ArmSubsystem, command),
                                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                                command2,
                                new WaitCommand(.25),
                                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                                new ArmSet2PtPath(m_ArmSubsystem,
                                                155, 233, 185, 227,
                                                40, 13, 100, 10,
                                                .2, .2, 0, .4, .2, 0,
                                                .1, .2, 0, .3, .1, 0,
                                                4, 4, 2, 2));

        }
}