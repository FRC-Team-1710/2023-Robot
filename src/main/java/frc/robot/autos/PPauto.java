package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PPauto extends SequentialCommandGroup {
    public PPauto(Swerve m_SwerveSubsystem) {

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("TEST", new PathConstraints(1.5, 1.5));

        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0),
                m_SwerveSubsystem::setModuleStates,
                false,
                m_SwerveSubsystem);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand);
    }
}
