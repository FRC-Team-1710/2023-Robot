// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSet2PtPath extends SequentialCommandGroup {
  /** Creates a new ArmSetPath. */

  private final ArmSetAngles pt1, pt2;

  /**
   * Moves the arm to a first set position, then a second position
   * 
   * @param subsystem ArmSubsystem is needed to run this
   * @param Hangle1   first set angle for the lower arm
   * @param Uangle1   first set angle for the upper arm
   * @param Hangle2   second angle for the lower arm
   * @param Uangle2   second angle for the upper arm
   * @param hdf1      speed to divide the lower arm for first point
   * @param udf1      speed to divide the upper arm for first point
   * @param hdf2      speed to divide the lower arm for second point
   * @param udf2      speed to divide the upper arm for second point
   */
  public ArmSet2PtPath(ArmSubsystem subsystem,
      double Hangle1, double Uangle1, double Hangle2, double Uangle2,
      double hdf1, double udf1, double hdf2, double udf2,
      double hp1, double hi1, double hd1, double up1, double ui1, double ud1,
      double hp2, double hi2, double hd2, double up2, double ui2, double ud2,
      double hsr1, double usr1, double hsr2, double usr2) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    pt1 = new ArmSetAngles(subsystem, Hangle1, Uangle1, hdf1, udf1, hp1, hi1, hd1, up1, ui1, ud1, hsr1, usr1);
    pt2 = new ArmSetAngles(subsystem, Hangle2, Uangle2, hdf2, udf2, hp2, hi2, hd2, up2, ui2, ud2, hsr2, usr2);
    addCommands(
        pt1,
        pt2);
  }
}
