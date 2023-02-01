// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public static CANSparkMax hm1, hm2, hm3, hm4, um1, um2, um3, um4;
  public static DutyCycleEncoder humerus_encoder, ulna_encoder;
  public static PIDController hPID, uPID;

  final double humerusLength = 26.5;
  final double ulnaLength = 41;

  final double hkP = 0.05;
  final double hkI = 0.01;
  final double hkD = 0;

  final double ukP = .05;
  final double ukI = 0.01;
  final double ukD = 0;

  public ArmSubsystem() {
    hm1 = new CANSparkMax(30, MotorType.kBrushless);
    hm2 = new CANSparkMax(31, MotorType.kBrushless);
    hm3 = new CANSparkMax(32, MotorType.kBrushless);
    hm4 = new CANSparkMax(33, MotorType.kBrushless);
    um1 = new CANSparkMax(34, MotorType.kBrushless);
    um2 = new CANSparkMax(35, MotorType.kBrushless);

    humerus_encoder = new DutyCycleEncoder(9);
    ulna_encoder = new DutyCycleEncoder(8);

    hPID = new PIDController(hkP, hkI, hkD);
    uPID = new PIDController(ukP, ukI, ukD);

    hm1.restoreFactoryDefaults();
    hm2.restoreFactoryDefaults();
    hm3.restoreFactoryDefaults();
    hm4.restoreFactoryDefaults();
    um1.restoreFactoryDefaults();
    um2.restoreFactoryDefaults();

    hm2.follow(hm1, false);
    hm3.follow(hm1, true);
    hm4.follow(hm1, true);
    um2.follow(um1, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hangle", getHPos());
    SmartDashboard.putNumber("uangle", getUPos());
  }


  public void setAngles(double hangle, double uangle) {
    hm1.set(hPID.calculate(humerus_encoder.get()*360, hangle)/10);
    um1.set(uPID.calculate(ulna_encoder.get()*360, uangle)/5);
  }

  public void zeroArm() {
    humerus_encoder.reset();
    ulna_encoder.reset();
  }

  public double getHPos() {
    return ((humerus_encoder.get()*360) % 360);
  }

  public double getUPos() {
    return ((ulna_encoder.get()*360) % 360);
  }
}