// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmSet2PtPath;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TheSloppyTopper3 extends ParallelCommandGroup {
    /** Creates a new TheSloppyTopper3. */
    public TheSloppyTopper3(ArmSubsystem m_ArmSubsystem, FollowPathWithEvents command) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ArmSet2PtPath(m_ArmSubsystem,
                        155, 233, 185, 227,
                        30, 10, 40, 4,
                        .2, .2, 0, .4, .2, 0,
                        .1, .2, 0, .3, .1, 0,
                        4, 4, 2, 2),
                command);
    }
}