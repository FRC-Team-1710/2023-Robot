package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.commands.autoBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Jhoppa extends SequentialCommandGroup {
    public Jhoppa(Swerve m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem,
            PneumaticSubsystem m_PneumaticSubsystem) {

        PathPlannerTrajectory trajectory1;

        double rotP = 1.25;
        double rotD = 0.06;
        double driveP = 1.5;
        double driveD = 0.02;

        Pose2d initialPose;

        /*if (DriverStation.getAlliance() == Alliance.Red) {
            trajectory1 = PathPlanner.loadPath("KepychKapper Red", new PathConstraints(2, 2));
            initialPose = new Pose2d(14.74, 4.95, new Rotation2d(0));
        } else {*/
            trajectory1 = PathPlanner.loadPath("jhoppa", new PathConstraints(2, 2));
            initialPose = trajectory1.getInitialPose();
        //}

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

        addCommands(
                new InstantCommand(() -> m_SwerveSubsystem.resetModulesToAbsolute()),
                new InstantCommand(() -> m_SwerveSubsystem.setGyro(0)),
                new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(initialPose)),
                //new WaitCommand(1),
                //new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsForward()),
                new WaitCommand(.1),
                new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 179, 115,
                20, 15, 70, 3,
                .35, .1, 0, .9, .7, 0,
                .35, .1, 0, .35, .4, 0,
                5, 4, 6, 4.5).raceWith(new WaitCommand(5)),
                new InstantCommand(() -> m_PneumaticSubsystem.SetTwoSolenoidsReverse()),
                path1.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 115, 268,
                30, 15, 20, 7,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 3, 3)),
                new autoBalance(m_SwerveSubsystem));
    }
}
