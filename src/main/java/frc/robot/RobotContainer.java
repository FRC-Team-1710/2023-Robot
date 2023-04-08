package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Controller Buttons */
    private final int MLSYAxis = XboxController.Axis.kLeftY.value;
    private final int MRSYAxis = XboxController.Axis.kRightY.value;
    private final int RT = XboxController.Axis.kRightTrigger.value;
    private final int LT = XboxController.Axis.kLeftTrigger.value;

    private final JoystickButton DstartButton = new JoystickButton(Dcontroller, XboxController.Button.kStart.value);
    private final JoystickButton DaButton = new JoystickButton(Dcontroller, XboxController.Button.kA.value);
    private final JoystickButton DbButton = new JoystickButton(Dcontroller, XboxController.Button.kB.value);
    private final JoystickButton DxButton = new JoystickButton(Dcontroller, XboxController.Button.kX.value);
    private final JoystickButton DyButton = new JoystickButton(Dcontroller, XboxController.Button.kY.value);
    private final JoystickButton DleftBumper = new JoystickButton(Dcontroller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton DrightBumper = new JoystickButton(Dcontroller, XboxController.Button.kRightBumper.value);
    private final JoystickButton DleftStick = new JoystickButton(Dcontroller, XboxController.Button.kLeftStick.value);
    private final JoystickAnalogButton DrightTrigger = new JoystickAnalogButton(Dcontroller, RT);

    private final JoystickButton MstartButton = new JoystickButton(Mcontroller, XboxController.Button.kStart.value);
    private final JoystickButton MselectButton = new JoystickButton(Mcontroller, XboxController.Button.kBack.value);

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
    private final LedSubsystem m_LedSubsystem = new LedSubsystem();

    private final SendableChooser<Command> commandChooser = new SendableChooser<>();

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
                        () -> DleftBumper.getAsBoolean(),
                        () -> DaButton.getAsBoolean(),
                        () -> DbButton.getAsBoolean(),
                        m_LedSubsystem));

        m_ArmSubsystem.setDefaultCommand(new moveArmWithTheSticks(
                m_ArmSubsystem,
                () -> Mcontroller.getRawAxis(MLSYAxis),
                () -> Mcontroller.getRawAxis(MRSYAxis),
                () -> MrightBumper.getAsBoolean()));

        // m_LedSubsystem.setDefaultCommand(new LedCommand(m_LedSubsystem));

        commandChooser.setDefaultOption("Kapper (Top)",
                new KepychKapper(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_PneumaticSubsystem));
        commandChooser.addOption("Topper (Bot)", new TheSloppyTopper(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem,
                m_PneumaticSubsystem, m_VisionSubsystem));
        commandChooser.addOption("Mid", new HighScore(m_SwerveSubsystem, m_IntakeSubsystem, 
                m_ArmSubsystem, m_PneumaticSubsystem));
        commandChooser.addOption("No Arm DO NOT USE", new STNoArm(m_SwerveSubsystem, m_IntakeSubsystem));
        SmartDashboard.putData("Auto Selection", commandChooser);

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
        DxButton.whileTrue(new autoBalance(m_SwerveSubsystem));
        //DaButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, 0));
        //DxButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, -1));
        //DbButton.onTrue(new TheSightChallenge(m_VisionSubsystem, m_SwerveSubsystem, 1));
        // DrightTrigger.whileTrue(new IntakeWithVision(m_IntakeSubsystem,
        // m_SwerveSubsystem, m_VisionSubsystem));

        // double hos = 0;
        // double uos = 0;

        MyButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
        83, 277, 179, 104,
        30, 15, 80, 20,
        .3, .1, 0, .6, .25, 0,
        .35, .1, 0, .35, .1, 0,
        5, 5, 2.5, 4.5)); // high

        /* 
        MxButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
        140, 115, 236, -26,
        30, 15, 60, 25,
        .3, .1, 0, .6, .25, 0,
        .35, .1, 0, .35, .1, 0,
        9, 10, 4, 6)); // high

        */
        MbButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 128.5, 194,
                30, 15, 50, 20,
                .3, .1, 0, .6, .2, 0,
                .25, .1, 0, .25, .1, 0,
                9, 10, 7, 7)); // mid

        MaButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 115, 268,
                30, 15, 20, 7,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 3, 3)); // intake
/* 
        MxButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
        83, 173, 170, 15,
        40, 30, 80, 35,
        .3, .1, 0, .6, .25, 0,
        .35, .1, 0, .35, .1, 0,
        7, 10, 2, 4)); // test
*/
        MleftBumper.onTrue(new InstantCommand(() -> m_PneumaticSubsystem.ToggleTwoSolenoids()));
        MstartButton.onTrue(new stopTheArmTweakin(m_ArmSubsystem)); // stop arm
        MselectButton.onTrue(new InstantCommand(() -> m_LedSubsystem.ToggleBlink()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(m_SwerveSubsystem, testPath)
        // return new KepychKapper(m_SwerveSubsystem, m_IntakeSubsystem,
        // m_ArmSubsystem, m_PneumaticSubsystem);

        // return new TheSloppyTopper(m_SwerveSubsystem, m_IntakeSubsystem,
        // m_ArmSubsystem, m_PneumaticSubsystem, m_VisionSubsystem);
        // return new ScoreNBalance(m_SwerveSubsystem, m_IntakeSubsystem,
        // m_ArmSubsystem, m_PneumaticSubsystem);

        return commandChooser.getSelected();
    }
}
