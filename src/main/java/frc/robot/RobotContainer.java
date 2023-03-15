package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
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

    private final int DRT = XboxController.Axis.kRightTrigger.value;
    private final JoystickButton DstartButton = new JoystickButton(Dcontroller, XboxController.Button.kStart.value);
    private final JoystickButton DaButton = new JoystickButton(Dcontroller, XboxController.Button.kA.value);
    private final JoystickButton DbButton = new JoystickButton(Dcontroller, XboxController.Button.kB.value);
    private final JoystickButton DxButton = new JoystickButton(Dcontroller, XboxController.Button.kX.value);
    private final JoystickButton DyButton = new JoystickButton(Dcontroller, XboxController.Button.kY.value);
    private final JoystickButton DrightTrigger = new JoystickButton(Dcontroller,
            XboxController.Axis.kRightTrigger.value);
    private final JoystickButton DleftBumper = new JoystickButton(Dcontroller,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton DrightBumper = new JoystickButton(Dcontroller,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton DrightStick = new JoystickButton(Dcontroller,
            XboxController.Button.kRightStick.value);
    private final JoystickButton DleftStick = new JoystickButton(Dcontroller,
            XboxController.Button.kLeftStick.value);

    private final JoystickButton MstartButton = new JoystickButton(Mcontroller, XboxController.Button.kStart.value);
    private final JoystickButton MaButton = new JoystickButton(Mcontroller, XboxController.Button.kA.value);
    private final JoystickButton MbButton = new JoystickButton(Mcontroller, XboxController.Button.kB.value);
    private final JoystickButton MxButton = new JoystickButton(Mcontroller, XboxController.Button.kX.value);
    private final JoystickButton MyButton = new JoystickButton(Mcontroller, XboxController.Button.kY.value);
    private final JoystickButton MleftBumper = new JoystickButton(Mcontroller,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton MrightBumper = new JoystickButton(Mcontroller,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton MrightStick = new JoystickButton(Mcontroller,
            XboxController.Button.kRightStick.value);
    private final JoystickButton MleftStick = new JoystickButton(Mcontroller,
            XboxController.Button.kLeftStick.value);

    /* Subsystems */
    private final Swerve m_SwerveSubsystem = new Swerve();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
    private final LedSubsystem m_LedSubsystem = new LedSubsystem();

    /* Trajectories */
    Trajectory testPath;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        m_SwerveSubsystem,
                        () -> -Dcontroller.getRawAxis(translationAxis),
                        () -> -Dcontroller.getRawAxis(strafeAxis),
                        () -> -Dcontroller.getRawAxis(rotationAxis) * .8,
                        () -> DleftStick.getAsBoolean()));

        m_LedSubsystem.SetAllianceColor();

        m_IntakeSubsystem.setDefaultCommand(
                new IntakeCommand(
                        m_IntakeSubsystem,
                        () -> DrightBumper.getAsBoolean(),
                        () -> DleftBumper.getAsBoolean()));

        m_ArmSubsystem.setDefaultCommand(new moveArmWithTheSticks(
                m_ArmSubsystem,
                () -> Mcontroller.getRawAxis(MLSYAxis), () -> Mcontroller.getRawAxis(MRSYAxis),
                () -> MrightBumper.getAsBoolean()));

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
        DyButton.whileTrue(new IntakeWithVision(m_IntakeSubsystem, m_SwerveSubsystem,
                m_VisionSubsystem));
        DaButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, 0));
        DxButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, -1));
        DbButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, 1));
        // DrightTrigger.whileTrue(new IntakeWithVision(m_IntakeSubsystem,
        // m_SwerveSubsystem, m_VisionSubsystem));

        double hos = 0;
        double uos = 0;

        MyButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                138, 225, 238.5, 63,
                40, 30, 70, 10,
                .3, .1, 0, .6, .25, 0,
                .35, .1, 0, .35, .1, 0,
                5, 7, 2, 4)); // high

        MbButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                143.7, 225, 193.7, 149,
                25, 40, 70, 15,
                .3, .1, 0, .6, .2, 0,
                .25, .1, 0, .25, .1, 0,
                7, 10, 2, 4)); // mid

        MaButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                155, 233, 185, 227,
                40, 13, 60, 6,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                4, 4, 2, 2)); // intake
                
        MxButton.onTrue(new ArmSetAngles(m_ArmSubsystem,
                178
                , 224.7,
                40, 13,
                .1, .1, 0,
                .3, .1, 0,
                2, 3)); // test

        MleftBumper.onTrue(new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()));
        MstartButton.onTrue(new stopTheArmTweakin(m_ArmSubsystem)); // stop arm

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(m_SwerveSubsystem, testPath)
         return new KepychKapper(m_SwerveSubsystem, m_IntakeSubsystem,
         m_ArmSubsystem, m_PneumaticSubsystem);

        //return new TheSloppyTopper(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_PneumaticSubsystem, m_VisionSubsystem);
    }
}
