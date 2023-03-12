// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {

    private Swerve swerveSubsystem;
    private VisionSubsystem visionSubsystem;
    private final PIDController xPidController, yPidController, aPidController;
    public double XO, YO;
    public boolean done;

    /** Creates a new VisionCommand. */
    public VisionCommand(VisionSubsystem m_VisionSubsystem, Swerve m_SwerveSubsystem, double yo) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_VisionSubsystem);

        swerveSubsystem = m_SwerveSubsystem;
        visionSubsystem = m_VisionSubsystem;

        YO = yo;

        xPidController = new PIDController(.3, .05, 0);
        yPidController = new PIDController(.3, .1, 0);
        aPidController = new PIDController(.05, .001, 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        String poseS;
        Pose2d pose;
        if (visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).isPresent()) {
            poseS = visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).get().estimatedPose.toPose2d()
                    .toString();
            pose = visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).get().estimatedPose.toPose2d();
        } else {
            poseS = swerveSubsystem.getPose().toString();
            pose = swerveSubsystem.getPose();
        }
        SmartDashboard.putString("pose", poseS);
        if (visionSubsystem.irisHasTarget()) {
            double currentX = pose.getX();
            double currentY = pose.getY();
            double currentA = pose.getRotation().getDegrees();
            int tagID = visionSubsystem.getTagID();
            int xs, ys;
            if (tagID > 4) {
                xs = 1;
            } else {
                xs = -1;
            }
            if (tagID > 4) {
                ys = -1;
            } else {
                ys = 1;
            }

            double xsp = visionSubsystem.getIDX(tagID) + (1 * xs);
            double ysp = visionSubsystem.getIDY(tagID) + (.8 * YO * ys);

            double vx = xPidController.calculate(currentX, xsp);
            double vy = yPidController.calculate(currentY, ysp);
            double va = aPidController.calculate(currentA, 0);

            swerveSubsystem.drive(new Translation2d(vx, vy), va, false, false);
            if (Math.abs(xsp - currentX) < .1 && Math.abs(ysp - currentY) < .1 && Math.abs(0 - currentA) < 2) {
                done = true;
            }

        } else {
            swerveSubsystem.drive(new Translation2d(0, 0), 0, false, false);
            done = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (done) {
            return true;
        } else {
            return false;
        }
    }
}
