// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {

    private Swerve swerveSubsystem;
    private VisionSubsystem visionSubsystem;

    /** Creates a new VisionCommand. */
    public VisionCommand(VisionSubsystem m_VisionSubsystem, Swerve m_SwerveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_VisionSubsystem);
        swerveSubsystem = m_SwerveSubsystem;
        visionSubsystem = m_VisionSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       // if (visionSubsystem.irisHasTarget()) {
         //   if (!swerveSubsystem.odometryDisabled) {
        //        swerveSubsystem.odometryDisabled = true;
        //    }
            // method call to vision to get pose
            
            // swerveSubsystem.resetOdometry(^);
       // } else {
       //     swerveSubsystem.odometryDisabled = false;
       // }
      /*  String pose;
       if(visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).isPresent()){
       pose = visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).get().estimatedPose.toPose2d().toString();
       } if (visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).isPresent() == false) {
        pose = swerveSubsystem.getPose().toString();
       } else {
        pose = "0";
       }*/
        
       
      // SmartDashboard.putString("pose", pose);

      // SmartDashboard.putString("pose", visionSubsystem.getEstimatedGlobalPose(swerveSubsystem.getPose()).get().estimatedPose.toPose2d().toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
