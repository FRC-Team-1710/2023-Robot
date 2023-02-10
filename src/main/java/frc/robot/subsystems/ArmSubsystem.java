// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public static CANSparkMax hm1, hm2, hm3, hm4, um1, um2;
  public static DutyCycleEncoder humerus_encoder, ulna_encoder;
  public static PIDController hPID, uPID;

  final double hkP = 0.25;
  final double hkI = 0.1;
  final double hkD = 0;

  final double ukP = 0.25;
  final double ukI = 0.1;
  final double ukD = 0;

  public ArmSubsystem() {
    hm1 = new CANSparkMax(Constants.humerusMotor1ID, MotorType.kBrushless);
    hm2 = new CANSparkMax(Constants.humerusMotor2ID, MotorType.kBrushless);
    hm3 = new CANSparkMax(Constants.humerusMotor3ID, MotorType.kBrushless);
    hm4 = new CANSparkMax(Constants.humerusMotor4ID, MotorType.kBrushless);
    um1 = new CANSparkMax(Constants.ulnaMotor1ID, MotorType.kBrushless);
    um2 = new CANSparkMax(Constants.ulnaMotor2ID, MotorType.kBrushless);

    humerus_encoder = new DutyCycleEncoder(Constants.humerusEncoderID);
    ulna_encoder = new DutyCycleEncoder(Constants.ulnaEncoderID);

    hPID = new PIDController(hkP, hkI, hkD);
    uPID = new PIDController(ukP, ukI, ukD);

    hm1.restoreFactoryDefaults();
    hm2.restoreFactoryDefaults();
    hm3.restoreFactoryDefaults();
    hm4.restoreFactoryDefaults();
    um1.restoreFactoryDefaults();
    um2.restoreFactoryDefaults();

    hm1.setIdleMode(IdleMode.kBrake);
    hm2.setIdleMode(IdleMode.kBrake);
    hm3.setIdleMode(IdleMode.kBrake);
    hm4.setIdleMode(IdleMode.kBrake);
    um1.setIdleMode(IdleMode.kBrake);
    um2.setIdleMode(IdleMode.kBrake);

    hm2.follow(hm1, false);
    hm3.follow(hm1, true);
    hm4.follow(hm1, true);
    um2.follow(um1, false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hangle", getHPos());
    SmartDashboard.putNumber("uangle", getUPos());
    SmartDashboard.putNumber("hum speed", hm1.get());
    SmartDashboard.putNumber("uln speed", um1.get());
  }

  public void setAngles(double hangle, double uangle) {
    hm1.set(hPID.calculate(humerus_encoder.get()*360, hangle)/25);
    um1.set(uPID.calculate(ulna_encoder.get()*360, uangle)/25);
  }

  public void zeroArm() {
    humerus_encoder.reset();
    ulna_encoder.reset();
  }

  public double getHPos() {
    return ((humerus_encoder.get()*360));
  }

  public double getUPos() {
    return ((ulna_encoder.get()*360));
  }
}