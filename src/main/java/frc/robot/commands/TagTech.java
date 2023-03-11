// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TagTech extends ParallelDeadlineGroup {

  /** Creates a new TagTech. */
  
  public TagTech(Swerve subsystem1, VisionSubsystem subsystem2) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new TagLineup(subsystem1, subsystem2));
    addCommands(new VisionCommand(subsystem2, subsystem1));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
