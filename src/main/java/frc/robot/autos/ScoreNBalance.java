package frc.robot.autos;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreNBalance extends SequentialCommandGroup {
    public ScoreNBalance(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem) {

        // PathPlannerTrajectory trajectory0 = PathPlanner.loadPath("Score 2 pt 1-1",
        // new PathConstraints(2, 2));
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("TopScorer1", new PathConstraints(2, 2));
        // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Score 2 pt 2", new
        // PathConstraints(3, 2.5));
        // PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("Score 2 pt 3", new
        // PathConstraints(3, 2.5));

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        /*
         * PPSwerveControllerCommand path0 = new PPSwerveControllerCommand(
         * trajectory0,
         * m_SwerveSubsystem::getPose,
         * Constants.Swerve.swerveKinematics,
         * new PIDController(driveP, 0, D2),
         * new PIDController(driveP, 0, D2),
         * new PIDController(P, 0, D),
         * m_SwerveSubsystem::setModuleStates,
         * false,
         * m_SwerveSubsystem);
         */

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new InstantCommand(() -> m_IntakeSubsystem.spin(.5)));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

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

        FollowPathWithEvents command = new FollowPathWithEvents(
                path1,
                trajectory1.getMarkers(),
                eventMap);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
                // new ArmSet2PtPath(m_ArmSubsystem, 265, 230, 354.8, 76, 25, 40, 80, 40),
                // new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                command);
    }
}
