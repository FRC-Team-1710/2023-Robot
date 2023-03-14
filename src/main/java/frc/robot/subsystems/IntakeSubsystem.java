// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX intake;
  boolean spinning = false;

  public IntakeSubsystem() {
    intake = new TalonFX(50, "carnivore uno");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleSpin() {
    if (spinning == false) {
      intake.set(TalonFXControlMode.PercentOutput, .5);
      spinning = true;
    }
    if (spinning == true) {
      intake.set(TalonFXControlMode.PercentOutput, 0);
      spinning = false;
    }

  }

  public void spin(double speed) {
    intake.set(TalonFXControlMode.PercentOutput, speed);
  }
}
