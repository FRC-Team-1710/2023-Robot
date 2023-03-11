package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.VisionSubsystem;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translation;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private BooleanSupplier robotCentric;

    public TeleopSwerve(Swerve m_SwerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = m_SwerveSubsystem;
        addRequirements(m_SwerveSubsystem);
       

        this.translation = translationSup;
        this.strafe = strafeSup;
        this.rotation = rotationSup;
        this.robotCentric = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(Math.copySign(Math.pow(translationVal, 2), translationVal), Math.copySign(Math.pow(strafeVal, 2), strafeVal)).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );

        
    }
}