// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KepychKapper2 extends ParallelCommandGroup {
    /** Creates a new ScoreNBalance2. */
    public KepychKapper2(ArmSubsystem m_ArmSubsystem, Swerve m_SwerveSubsystem, FollowPathWithEvents command) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                command.deadlineWith(new ArmSet2PtPath(m_ArmSubsystem,
                146, 180, 185, 178,
                40, 13, 50, 5,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                4, 4, 2, 2))); 
    }
}
