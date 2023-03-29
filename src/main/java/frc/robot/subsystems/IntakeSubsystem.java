// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX thruIntake;
  TalonFX overIntake;
  TalonFX jointL;
  TalonFX jointR;
  boolean spinning = false;
  boolean up;

  public IntakeSubsystem() {
    thruIntake = new TalonFX(50, "carnivore uno");
    overIntake = new TalonFX(53);
    jointL = new TalonFX(51);
    jointR = new TalonFX(52);

    jointL.configFactoryDefault();
    jointR.configFactoryDefault();

    jointL.setNeutralMode(NeutralMode.Brake);
    jointR.setNeutralMode(NeutralMode.Brake);

    jointR.setInverted(true);
    overIntake.setInverted(true);

    jointL.setSelectedSensorPosition(0);
    jointR.setSelectedSensorPosition(0);

    jointL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    jointR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    jointL.config_kP(0, 0.08);
    jointR.config_kP(0, 0.08);

    jointL.config_kI(0, 0.0001);
    jointR.config_kI(0, 0.0001);

    jointL.config_kD(0, 5);
    jointR.config_kD(0, 5);

    jointL.configClosedloopRamp(.5);
    jointR.configClosedloopRamp(.5);

    jointL.selectProfileSlot(0, 0);
    jointR.selectProfileSlot(0, 0);

    jointL.configAllowableClosedloopError(0, 50);
    jointR.configAllowableClosedloopError(0, 50);

    //jointL.follow(jointR);

    up = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Position", jointL.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", jointR.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Intake upo", up);
  }

  public void toggleSpin() {
    if (spinning == false) {
      thruIntake.set(TalonFXControlMode.PercentOutput, .5);
      spinning = true;
    }
    if (spinning == true) {
      thruIntake.set(TalonFXControlMode.PercentOutput, 0);
      spinning = false;
    }

  }

  public void spin(double speed) {
    thruIntake.set(TalonFXControlMode.PercentOutput, speed);
    if (!up){
      overIntake.set(TalonFXControlMode.PercentOutput, speed);
    }
      if (speed > 0 && up){
        jointR.set(TalonFXControlMode.Position, 8700);
        jointL.set(TalonFXControlMode.Position, 8900);
        up = false;
      } else if (speed == 0 && !up){
        jointR.set(TalonFXControlMode.Position, -200);
        jointL.set(TalonFXControlMode.Position, -300);
        up = true;
      } else if (speed < 0 && !up){
        jointR.set(TalonFXControlMode.Position, -200);
        jointL.set(TalonFXControlMode.Position, -300);
        up = true;
      }
    }

    public void spinInside(double speed){
      thruIntake.set(TalonFXControlMode.PercentOutput, speed);
    }
}
