package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick controller = new Joystick(0);
    //private final Joystick driveController = new Joystick(0);
    //private final Joystick mechController = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Controller Buttons */
    //private final JoystickButton robotCentric = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton startButton = new JoystickButton(controller, XboxController.Button.kStart.value);
    private final JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(controller, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(controller, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(controller, XboxController.Button.kY.value);
    private final JoystickButton leftBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve m_SwerveSubsystem = new Swerve();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();
    //private final LedSubsystem m_LedSubsystem = new LedSubsystem();

    /* Trajectories */
    Trajectory testPath;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                m_SwerveSubsystem, 
                () -> controller.getRawAxis(translationAxis)*.75, 
                () -> controller.getRawAxis(strafeAxis) * .75, 
                () -> controller.getRawAxis(rotationAxis) *.75, 
                () -> leftBumper.getAsBoolean()
            )
        );

        try {
            testPath = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(
                    "pathplanner/generatedJSON/TEST.wpilib.json"));

         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
         }

         //m_LedSubsystem.setDefaultCommand(new LedCommand(m_LedSubsystem));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Buttons */
        //startButton.onTrue(new InstantCommand(() -> m_ArmSubsystem.zeroArm()));
        aButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 30 + 25 , 230 ));
        bButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 30 + 30, 220 + 60));
        yButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 30 + 30, 220 + 80));
        xButton.onTrue(new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()));
        startButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(m_SwerveSubsystem, testPath);
        return new PPauto(m_SwerveSubsystem);
    }
}
