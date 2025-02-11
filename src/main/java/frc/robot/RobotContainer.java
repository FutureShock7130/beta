package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStruct;
public class RobotContainer {
    
    private CommandXboxController m_Controller;
    private Joystick joystick;
    
    private final Elevator m_elevator;
    private final Grabber m_grabber;
    private final Intake m_intake;
    private final SuperStruct m_SuperStruct;

    public RobotContainer() {
        m_elevator = Elevator.getInstance();
        m_grabber = Grabber.getInstance();
        m_intake = Intake.getInstance();
        m_SuperStruct = SuperStruct.getInstance();
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }

    
}
