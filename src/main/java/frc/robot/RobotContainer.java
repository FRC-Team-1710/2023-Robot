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
    private final JoystickButton DleftBumper = new JoystickButton(Dcontroller,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton DrightBumper = new JoystickButton(Dcontroller,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton DleftStick = new JoystickButton(Dcontroller,
            XboxController.Button.kLeftStick.value);
    private final JoystickAnalogButton DrightTrigger = new JoystickAnalogButton(Dcontroller, RT);

    private final JoystickButton MstartButton = new JoystickButton(Mcontroller, XboxController.Button.kStart.value);
    private final JoystickButton MselectButton = new JoystickButton(Mcontroller, XboxController.Button.kBack.value);

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
                new KepychKapper(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem,
                        m_PneumaticSubsystem));
        commandChooser.addOption("Topper (DO NOT USE)",
                new TheSloppyTopper(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem,
                        m_PneumaticSubsystem, m_VisionSubsystem));
        commandChooser.addOption("Jhoppin (Mid cone)", new Jhoppin(m_SwerveSubsystem, m_IntakeSubsystem,
                m_ArmSubsystem, m_PneumaticSubsystem));
        commandChooser.addOption("Jhoppa (Mid cube)",
                new Jhoppa(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_PneumaticSubsystem));
        commandChooser.addOption("Sloppy Three (DO NOT USE)", new SloppyThree(m_SwerveSubsystem, m_IntakeSubsystem, m_PneumaticSubsystem, m_VisionSubsystem));
        commandChooser.addOption("Daly Droppa (Bot)", new DalyDroppa(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_PneumaticSubsystem));
        commandChooser.addOption("Daly Droppa (Top)", new DalyDroppa2(m_SwerveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_PneumaticSubsystem));
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
        //DyButton.onTrue(new the28thAmmendments(m_VisionSubsystem, m_SwerveSubsystem));
        /*
         * DyButton.whileTrue(new IntakeWithVision(m_IntakeSubsystem, m_SwerveSubsystem,
         * m_VisionSubsystem));
         * DxButton.onTrue(new IntakeVisionAuto(m_IntakeSubsystem, m_SwerveSubsystem,
         * m_VisionSubsystem));
         * 
         */
        // DxButton.whileTrue(new autoBalance(m_SwerveSubsystem));

        MyButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                79, 275, 179, 115,
                10, 9, 50, 4,
                .35, .1, 0, .9, .7, 0,
                .35, .1, 0, .35, .4, 0,
                5, 4, 6, 4.5)); // high

        MbButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                79, 275, 128.5, 204,
                10, 9, 50, 7,
                .3, .1, 0, .6, .2, 0,
                .25, .1, 0, .25, .1, 0,
                9, 10, 7, 7)); // mid

        MaButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
                83, 277, 115, 271,
                30, 15, 20, 3,
                .2, .2, 0, .4, .2, 0,
                .1, .2, 0, .3, .1, 0,
                7, 5, 4, 3)); // intake
        /*
         * MxButton.onTrue(new ArmSet2PtPath(m_ArmSubsystem,
         * 83, 173, 170, 15,
         * 40, 30, 80, 35,
         * .3, .1, 0, .6, .25, 0,
         * .35, .1, 0, .35, .1, 0,
         * 7, 10, 2, 4)); // test
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
        return commandChooser.getSelected();
    }
}
