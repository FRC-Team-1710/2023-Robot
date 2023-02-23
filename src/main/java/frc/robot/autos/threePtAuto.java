package frc.robot.autos;

import frc.robot.Constants;
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

public class threePtAuto extends SequentialCommandGroup {
    public threePtAuto(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem) {

      PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("Score 2 pt 1", new PathConstraints(.5, .5));
      PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Score 2 pt 2", new PathConstraints(.5, .5));
      PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("Score 2 pt 3", new PathConstraints(.5, .5));

        PPSwerveControllerCommand path1 = new PPSwerveControllerCommand(
                trajectory1,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0),
                m_SwerveSubsystem::setModuleStates,
                true,
                m_SwerveSubsystem);

                PPSwerveControllerCommand path2 = new PPSwerveControllerCommand(
                trajectory2,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0),
                m_SwerveSubsystem::setModuleStates,
                true,
                m_SwerveSubsystem);


                PPSwerveControllerCommand path3 = new PPSwerveControllerCommand(
                trajectory3,
                m_SwerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0),
                m_SwerveSubsystem::setModuleStates,
                true,
                m_SwerveSubsystem);


        addCommands(
            new InstantCommand(() -> m_SwerveSubsystem.setGyro(180)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
                new InstantCommand(() -> m_IntakeSubsystem.spin(-.3)),
                new WaitCommand(1),
                new InstantCommand(() -> m_IntakeSubsystem.spin(0)),
                path1,
                new InstantCommand(() -> m_IntakeSubsystem.spin(.3)),
                path2,
                new WaitCommand(1),
                path3,
                new InstantCommand(() -> m_IntakeSubsystem.spin(-.3)),
                new WaitCommand(1),
                new InstantCommand(() -> m_IntakeSubsystem.spin(0)));
    }
}
