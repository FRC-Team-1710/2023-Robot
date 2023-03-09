package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

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

public class threePtAuto extends SequentialCommandGroup {
    public threePtAuto(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem) {

        PathPlannerTrajectory trajectory0 = PathPlanner.loadPath("Score 2 pt 1-1", new PathConstraints(2, 2));
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Score 2 pt 1-2", new PathConstraints(2, 2));
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Score 2 pt 2", new PathConstraints(3, 2.5));
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("Score 2 pt 3", new PathConstraints(3, 2.5));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new InstantCommand(() -> m_IntakeSubsystem.spin(.5)));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        /*PPSwerveControllerCommand path0 = new PPSwerveControllerCommand(
                trajectory0,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(driveP, 0, D2),
                new PIDController(driveP, 0, D2),
                new PIDController(P, 0, D),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);*/

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

                FollowPathWithEvents command2 = new FollowPathWithEvents(
                    path3,
                    trajectory3.getMarkers(),
                    eventMap);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory0.getInitialPose())),
                //new WaitCommand(.1),
              //  path0,
                new IntakeSpin(m_IntakeSubsystem, -.5),
                new WaitCommand(.65),
                new IntakeSpin(m_IntakeSubsystem, 0),
                command,
                path2,
                new IntakeSpin(m_IntakeSubsystem, -.5),
                new WaitCommand(.75),
                new IntakeSpin(m_IntakeSubsystem, 0),
                command2);
    }
}
