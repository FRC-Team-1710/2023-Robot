package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.autoBalance;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class KepychKapper extends SequentialCommandGroup {
    public KepychKapper(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem) {

        // PathPlannerTrajectory trajectory0 = PathPlanner.loadPath("Score 2 pt 1-1",
        // new PathConstraints(2, 2));
        // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Score 2 pt 2", new
        // PathConstraints(3, 2.5));
        // PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("Score 2 pt 3", new
        // PathConstraints(3, 2.5));

        PathPlannerTrajectory trajectory1;

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        Pose2d initialPose;

        if (DriverStation.getAlliance() == Alliance.Red){
                trajectory1 = PathPlanner.loadPath("KepychKapper Red", new PathConstraints(2, 2));
                initialPose = new Pose2d(14.74, 4.95, new Rotation2d(0));
        } else {
                trajectory1 = PathPlanner.loadPath("KepychKapper", new PathConstraints(2, 2));
                initialPose = trajectory1.getInitialPose();
        }

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

        /*
         * HashMap<String, Command> eventMap = new HashMap<>();
         * eventMap.put("Intake", new InstantCommand(() -> m_IntakeSubsystem.spin(.5)));
         * eventMap.put("Intook", new InstantCommand(() -> m_IntakeSubsystem.spin(0)));
         */

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new IntakeSpin(m_IntakeSubsystem, 0.5));
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
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsForward()),
                new WaitCommand(.1),
                new ArmSet2PtPath(m_ArmSubsystem,
        140, 115, 236, -30,
        30, 15, 60, 25,
        .3, .1, 0, .6, .25, 0,
        .35, .1, 0, .35, .1, 0,
        9, 10, 2.5, 4.5).raceWith(new WaitCommand(5.5)),
                new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()),
                //new WaitCommand(.1),
                //path1
                command.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                143, 123, 182, 116,
                30, 15, 20, 7,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 2, 2)),
                new autoBalance(m_SwerveSubsystem)
                //new KepychKapper2(m_ArmSubsystem, m_SwerveSubsystem, command)
        /*
          new ArmSet2PtPath(m_ArmSubsystem,
          155, 233, 185, 227,
          40*.85, 13*.85, 100, 10,
          .25, .2, 0, .4, .2, 0,
          .1, .2, 0, .3, .1, 0,
          4, 4, 2, 2),
          command
         */);
    }
}
