// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSet3PtPath extends SequentialCommandGroup {
  /** Creates a new ArmSetPath. */
  private final ArmSetAngles pt1, pt2, pt3;

  public ArmSet3PtPath(ArmSubsystem subsystem, double Hangle1, double Uangle1, double Hangle2, double Uangle2, double Hangle3, double Uangle3) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    pt1 = new ArmSetAngles(subsystem, Hangle1, Uangle1, 100, 6.5);
    pt2 = new ArmSetAngles(subsystem, Hangle2, Uangle2, 200, 6.5);
    pt3 = new ArmSetAngles(subsystem, Hangle3, Uangle3, 100, 6.5 );
    addCommands(
        pt1,
        pt2,
        pt3);
  }
}
