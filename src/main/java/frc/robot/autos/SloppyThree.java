package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.the28thAmmendments;
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

public class SloppyThree extends SequentialCommandGroup {
    public SloppyThree(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, PneumaticSubsystem m_PneumaticSubsystem, VisionSubsystem m_VisionSubsystem) {

        PathPlannerTrajectory trajectory1;
        PathPlannerTrajectory trajectory2;
        PathPlannerTrajectory trajectory3;

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        Pose2d initialPose;

        if (DriverStation.getAlliance() == Alliance.Red){
                trajectory1 = PathPlanner.loadPath("ST Red", new PathConstraints(1.5, 1.5));
                trajectory2 = PathPlanner.loadPath("ST2 Red", new PathConstraints(2.5, 2.5));
                trajectory3 = PathPlanner.loadPath("third piece red", new PathConstraints(1.5, 1.5));

                initialPose = new Pose2d(14.73, .5, new Rotation2d(0));
        } else {
                trajectory1 = PathPlanner.loadPath("ST", new PathConstraints(3, 3));
                trajectory2 = PathPlanner.loadPath("ST2", new PathConstraints(3, 3));
                trajectory3 = PathPlanner.loadPath("third piece", new PathConstraints(3, 3));
                
                initialPose = trajectory1.getInitialPose();
        }

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new IntakeSpin(m_IntakeSubsystem, 0.5));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));
        eventMap.put("Blind Man Sees", new the28thAmmendments(m_VisionSubsystem, m_SwerveSubsystem));


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

        PPSwerveControllerCommand path3 = new PPSwerveControllerCommand(
                    trajectory3,
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

        FollowPathWithEvents command2 = new FollowPathWithEvents(path2, trajectory2.getMarkers(), eventMap);

        FollowPathWithEvents command3 = new FollowPathWithEvents(path3, trajectory3.getMarkers(), eventMap);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsReverse()),
                new IntakeSpin(m_IntakeSubsystem, -0.7),
                //new WaitCommand(.7),
                //new IntakeSpin(m_IntakeSubsystem, 0),
                command,
                command2,
               // new WaitCommand(.15),
                new IntakeSpin(m_IntakeSubsystem, -0.55),
                new WaitCommand(1),
                new IntakeSpin(m_IntakeSubsystem, 0),
                command3);
    }
}