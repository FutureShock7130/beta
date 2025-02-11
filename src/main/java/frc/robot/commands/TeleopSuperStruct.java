package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.states.SuperStructState;
import frc.robot.subsystems.SuperStruct;

public class TeleopSuperStruct extends Command {
    private final SuperStruct m_superStruct;

    
    public TeleopSuperStruct(SuperStruct superStruct) {
        m_superStruct = superStruct;
        addRequirements(superStruct);
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {
        // Set to default state when starting teleop
        m_superStruct.setState(SuperStructState.DEFAULT);
    }

    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        
        m_superStruct.handleControllerInput();
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        // Return to safe default state when ending
        m_superStruct.setState(SuperStructState.DEFAULT);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        
        return false;
    }
} 