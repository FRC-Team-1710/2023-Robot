package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final JoystickButton DrightBumper = new JoystickButton(Dcontroller,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton DrightStick = new JoystickButton(Dcontroller, XboxController.Button.kRightStick.value);
    private final JoystickButton DleftStick = new JoystickButton(Dcontroller, XboxController.Button.kLeftStick.value);

    private final JoystickButton MstartButton = new JoystickButton(Mcontroller, XboxController.Button.kStart.value);
    private final JoystickButton MaButton = new JoystickButton(Mcontroller, XboxController.Button.kA.value);
    private final JoystickButton MbButton = new JoystickButton(Mcontroller, XboxController.Button.kB.value);
    private final JoystickButton MxButton = new JoystickButton(Mcontroller, XboxController.Button.kX.value);
    private final JoystickButton MyButton = new JoystickButton(Mcontroller, XboxController.Button.kY.value);
    private final JoystickButton MleftBumper = new JoystickButton(Mcontroller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton MrightBumper = new JoystickButton(Mcontroller,
            XboxController.Button.kRightBumper.value);
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
        //DrightBumper.onTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(.4)));
        DleftBumper.whileTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(-.6)));
        DbButton.whileTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(0)));
        //DrightBumper.onTrue(new IntakeWithVision(m_IntakeSubsystem, m_SwerveSubsystem, m_VisionSubsystem));
        DrightBumper.whileTrue(new InstantCommand(() -> m_IntakeSubsystem.spin(.6)));

        double hos = -28;
        double uos = 321;
        //MyButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 44 + hos, -14 + uos, 100, 10)); // change
        //MbButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 80 + hos, -76 + uos, 100, 10)); // change
        MyButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem, 104 + hos, -160 + uos, 44 + hos, -14 + uos));
        MbButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem, 104 + hos, -160 + uos, 80 + hos, -76 + uos));
        MaButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem, 104 + hos, -160 + uos, 70 + hos, -155 + uos)); // change
        MxButton.onTrue(new ArmSetAngles(m_ArmSubsystem, 104 + hos, -160 + uos, 200, 7)); // change
        MleftBumper.onTrue(new ArmSet2PtPath(m_ArmSubsystem, 104 + hos, -160 + uos, 113 + hos, -124 + uos));
        MrightBumper.onTrue(new ArmSet2PtPath(m_ArmSubsystem, 104 + hos, -160 + uos, 70 + hos, -155 + uos));
        //MrightBumper.onTrue(new ArmSetAngles(m_ArmSubsystem, 0 + 26, 180 - 85, 100, 10));
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
        return new threePtAuto(m_SwerveSubsystem, m_IntakeSubsystem);
    }
}
