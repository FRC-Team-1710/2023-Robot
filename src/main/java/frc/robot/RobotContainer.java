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
    private final Joystick Dcontroller = new Joystick(0);
    private final Joystick Mcontroller = new Joystick(1);
    // private final Joystick driveController = new Joystick(0);
    // private final Joystick mechController = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Controller Buttons */
    // private final JoystickButton robotCentric = new JoystickButton(controller,
    // XboxController.Button.kLeftBumper.value);
    private final int MLSYAxis = XboxController.Axis.kLeftY.value;
    private final int MRSYAxis = XboxController.Axis.kRightY.value;
    private final int MRT = XboxController.Axis.kRightTrigger.value;
    private final int MLT = XboxController.Axis.kLeftTrigger.value;
    private final JoystickButton DstartButton = new JoystickButton(Dcontroller, XboxController.Button.kStart.value);
    private final JoystickButton DaButton = new JoystickButton(Dcontroller, XboxController.Button.kA.value);
    private final JoystickButton DbButton = new JoystickButton(Dcontroller, XboxController.Button.kB.value);
    private final JoystickButton DxButton = new JoystickButton(Dcontroller, XboxController.Button.kX.value);
    private final JoystickButton DyButton = new JoystickButton(Dcontroller, XboxController.Button.kY.value);
    private final JoystickButton DleftBumper = new JoystickButton(Dcontroller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton DrightBumper = new JoystickButton(Dcontroller, XboxController.Button.kRightBumper.value);
    private final JoystickButton DrightStick = new JoystickButton(Dcontroller, XboxController.Button.kRightStick.value);
    private final JoystickButton DleftStick = new JoystickButton(Dcontroller, XboxController.Button.kLeftStick.value);

    private final JoystickButton MstartButton = new JoystickButton(Mcontroller, XboxController.Button.kStart.value);
    private final JoystickButton MaButton = new JoystickButton(Mcontroller, XboxController.Button.kA.value);
    private final JoystickButton MbButton = new JoystickButton(Mcontroller, XboxController.Button.kB.value);
    private final JoystickButton MxButton = new JoystickButton(Mcontroller, XboxController.Button.kX.value);
    private final JoystickButton MyButton = new JoystickButton(Mcontroller, XboxController.Button.kY.value);
    private final JoystickButton MleftBumper = new JoystickButton(Mcontroller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton MrightBumper = new JoystickButton(Mcontroller, XboxController.Button.kRightBumper.value);
    private final JoystickButton MrightStick = new JoystickButton(Mcontroller, XboxController.Button.kRightStick.value);
    private final JoystickButton MleftStick = new JoystickButton(Mcontroller, XboxController.Button.kLeftStick.value);

    /* Subsystems */
    private final Swerve m_SwerveSubsystem = new Swerve();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
    // private final LedSubsystem m_LedSubsystem = new LedSubsystem();

    /* Trajectories */
    Trajectory testPath;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        m_SwerveSubsystem,
                        () -> -Dcontroller.getRawAxis(translationAxis) * .75,
                        () -> -Dcontroller.getRawAxis(strafeAxis) * .75,
                        () -> -Dcontroller.getRawAxis(rotationAxis) * .75,
                        () -> DleftStick.getAsBoolean()));

        m_VisionSubsystem.setDefaultCommand(
            new VisionCommand(m_VisionSubsystem, m_SwerveSubsystem)
        );
        
        // m_LedSubsystem.setDefaultCommand(new LedCommand(m_LedSubsystem));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Buttons */
        DstartButton.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
        DrightBumper.onTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(.6)));
        DleftBumper.whileTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(-.6)));
        DbButton.whileTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(0)));

        MyButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 11, 180 + 9, 100, 10)); // change
        MbButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 53, 180 - 66, 100, 10)); // change
        MaButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 38, 180 - 138, 250, 7)); // change
        MxButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 98, 180 - 146, 100, 7)); // change
        MleftBumper.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 84, 180 - 93, 100, 10));
        MrightBumper.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 26, 180 - 85, 100, 10));
        MrightStick.onTrue(new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()));
        MstartButton.onTrue(new manualArm(m_ArmSubsystem, 0, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(m_SwerveSubsystem, testPath);
        return new AutoWithEvents(m_SwerveSubsystem, m_IntakeSubsystem);
    }
}
