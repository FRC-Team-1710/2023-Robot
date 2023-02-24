// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoWithEvents extends SequentialCommandGroup {
    /** Creates a new AutoWithEvents. */
    public AutoWithEvents(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Markers", new PathConstraints(2, 2));

        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new InstantCommand(() -> m_IntakeSubsystem.spin(.5)));
        eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

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

        FollowPathWithEvents command = new FollowPathWithEvents(
                swerveControllerCommand,
                trajectory.getMarkers(),
                eventMap);

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                command);
    }
}
