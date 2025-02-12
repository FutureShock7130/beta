package frc.FSLib2025.stateMachine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class StateMachine extends SubsystemBase {

    public static final RobotState Default = new RobotState();

    public static final RobotState Intake = new RobotState();

    public static final RobotState L1 = new RobotState();
    static {
        L1.ElevatorPosition = 0;
        L1.IntakePosition = 0;
        L1.IntakeState = 0;
    }

    public static final RobotState L2 = new RobotState();
    public static final RobotState L3 = new RobotState();
    public static final RobotState L4 = new RobotState();

    public static RobotState robotState = new RobotState();

    private final CommandXboxController commandController;
    private final XboxController controller;

    public StateMachine(CommandXboxController commandController) {
        this.commandController = commandController;
        this.controller = commandController.getHID();
    }

    public RobotState getRobotState() {
        return robotState;
    }

    @Override
    public void periodic() {
        if (controller.getAButtonPressed()) robotState = L1;
    }
}
