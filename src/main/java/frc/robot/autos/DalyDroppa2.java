package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DalyDroppa2 extends SequentialCommandGroup {
    public DalyDroppa2(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem) { 

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        Pose2d initialPose;

        PathPlannerTrajectory trajectory1;
        PathPlannerTrajectory trajectory2;

        if (DriverStation.getAlliance() == Alliance.Red) {
            trajectory1 = PathPlanner.loadPath("droppa red top", new PathConstraints(2, 2));
            trajectory2 = PathPlanner.loadPath("straight red top", new PathConstraints(2, 2));
        } else {
            trajectory1 = PathPlanner.loadPath("droppa blue top", new PathConstraints(2, 2));
            trajectory2 = PathPlanner.loadPath("straight blue top", new PathConstraints(2, 2));
        }

        initialPose = trajectory1.getInitialPose();

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new IntakeSpin(m_IntakeSubsystem, 0.5));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));
        eventMap.put("grab", new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()));
        eventMap.put("Arm Up", new ArmSet2PtPath(m_ArmSubsystem,
        79, 275, 128.5, 204,
        30, 15, 50, 25,
        .3, .1, 0, .6, .2, 0,
        .25, .1, 0, .25, .1, 0,
        9, 10, 7, 7));
        eventMap.put("Arm Down", new ArmSet2PtPath(m_ArmSubsystem,
        83, 277, 115, 271,
        30, 15, 20, 3,
        .2, .2, 0, .4, .2, 0,
        .1, .2, 0, .3, .1, 0,
        7, 5, 4, 3));

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

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                //new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsForward()),
                new WaitCommand(.1),
                new ArmSet2PtPath(m_ArmSubsystem,
                79, 275, 179, 115,
                10, 9, 50, 4,
                .35, .1, 0, .9, .7, 0,
                .35, .1, 0, .35, .4, 0,
                5, 4, 6, 4.5).raceWith(new WaitCommand(5)),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                command,
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                new WaitCommand(.5),
                path2.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 115, 271,
                30, 15, 20, 3,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 4, 3))
                );
    }
}
