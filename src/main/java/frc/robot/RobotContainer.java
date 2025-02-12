package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStruct;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;

public class RobotContainer {

    private CommandXboxController m_Controller;
    private Joystick joystick;

    private final Elevator m_elevator;
    private final Grabber m_grabber;
    private final Intake m_intake;
    private final SuperStruct m_SuperStruct;

    // private final Drive drive;
    private final Swerve m_swerve;
    private final Vision vision = new Vision();

    private final CommandXboxController controller = new CommandXboxController(0);

    private final SendableChooser<Command> autoChooser;
    
    private final TeleopSwerve teleopSwerve;

    public RobotContainer() {
        m_elevator = Elevator.getInstance();
        m_grabber = Grabber.getInstance();
        m_intake = Intake.getInstance();
        m_SuperStruct = SuperStruct.getInstance();
        m_swerve = new Swerve();
        teleopSwerve = new TeleopSwerve(m_swerve, controller);
        
        m_swerve.setDefaultCommand(teleopSwerve);

        // switch (Constants.currentMode) {
        //     case REAL:
        //         // Real robot, instantiate hardware IO implementations

        //         // drive = new Drive(
        //         //         new GyroIOPigeon2(),
        //         //         new Swerve(0),
        //         //         new Swerve(1),
        //         //         new Swerve(2),
        //         //         new Swerve(3),
        //         //         vision);

        //         // break;

        //     // case SIM:
        //     // // Sim robot, instantiate physics sim IO implementations
        //     // drive =
        //     // new Drive(
        //     // new GyroIO() {},
        //     // new ModuleIOSim(),
        //     // new ModuleIOSim(),
        //     // new ModuleIOSim(),
        //     // new ModuleIOSim());
        //     // break;

        //     default:
        //         // Replayed robot, disable IO implementations
        //         drive = new Drive(
        //                 new GyroIO() {
        //                 },
        //                 new ModuleIO() {
        //                 },
        //                 new ModuleIO() {
        //                 },
        //                 new ModuleIO() {
        //                 },
        //                 new ModuleIO() {
        //                 },
        //                 vision);
        //         break;
        // }

        // Set up auto routines

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("null", null);


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
