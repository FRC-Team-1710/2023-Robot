// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class autoBalance extends CommandBase {
    /** Creates a new autoBalance. */
    public Pigeon2 gyro;
    private final PIDController xPidController;
    private final Swerve m_SwerveSub;
    private Timer timer;

    public autoBalance(Swerve subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_SwerveSub = subsystem;
        addRequirements(subsystem);
        xPidController = new PIDController(.045, 0.0, 0.0);
        gyro = subsystem.gyro;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentA = gyro.getPitch();
        if (Math.abs(currentA) > 0.09) {
            timer.reset();
            double vx = xPidController.calculate(currentA, 0);
            m_SwerveSub.drive(new Translation2d(vx, 0), 0, false, true);
        } else {
            timer.start();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_SwerveSub.drive(new Translation2d(0, 0), 0, false, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timer.get() > 2) {
            return true;
        } else {
            return false;
        }
    }
}
