// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class IntakeCommand extends CommandBase {

  IntakeSubsystem intake;
  BooleanSupplier rightPressed;
  BooleanSupplier leftPressed;
  LedSubsystem LED;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier rBumper, BooleanSupplier lBumper, LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    addRequirements(intakeSubsystem);

    LED = ledSubsystem;
    rightPressed = rBumper;
    leftPressed = lBumper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rightPressed.getAsBoolean()){
      intake.spin(.5);
      LED.SetVisionPattern(true);
    } else if (leftPressed.getAsBoolean()){
      intake.spin(-.75);
      LED.SetVisionPattern(true);

    } else {
      intake.spin(0);
      LED.SetVisionPattern(false);
    }
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
