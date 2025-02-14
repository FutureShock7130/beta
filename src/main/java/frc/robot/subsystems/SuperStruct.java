// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.states.StateMachine;
import frc.robot.states.SuperStructState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;


//ele
//-2.8291015625 L2
//-4.19677734375 L3
//-4.8359375 L4

//grabber
//-0.312 L4
public class SuperStruct extends SubsystemBase {
  Elevator mElevator;
  Grabber mGrabber;
  Intake mIntake;
  AlgaeIntake mAlgaeIntake;
  StateMachine mStateMachine;
  SuperStructState mCommandedState;
  private final XboxController controller;
  private static final int CONTROLLER_PORT = 3; 
  private final Joystick buttonBoard1;
  private final Joystick buttonBoard2;

  private static SuperStruct mInstance = null;

  public static synchronized SuperStruct getInstance() {
      if (mInstance == null) {
          mInstance = new SuperStruct();
      }
      return mInstance;
  }
  /** Creates a new StateMachine. */
  public SuperStruct() {
    mElevator = Elevator.getInstance();
    mGrabber = Grabber.getInstance();
    mIntake = Intake.getInstance();
    mAlgaeIntake = AlgaeIntake.getInstance();
    mStateMachine = StateMachine.getInstance();
    mCommandedState = SuperStructState.DEFAULT;
    controller = new XboxController(CONTROLLER_PORT);
    buttonBoard1 = new Joystick(0);  // First port
    buttonBoard2 = new Joystick(1);  // Second port
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // First button board (Port 0) - State Controls
    new JoystickButton(buttonBoard1, 1)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L1),
            this
        ));

    new JoystickButton(buttonBoard1, 2)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L2),
            this
        ));

    new JoystickButton(buttonBoard1, 3)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L3),
            this
        ));

    new JoystickButton(buttonBoard1, 4)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.L4),
            this
        ));

    new JoystickButton(buttonBoard1, 5)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.TRAVEL),
            this
        ));

    new JoystickButton(buttonBoard1, 6)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.CS),
            this
        ));

    new JoystickButton(buttonBoard1, 7)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.PLACEMENT),
            this
        ));

    new JoystickButton(buttonBoard1, 8)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.DEFAULT),
            this
        ));

    // Second button board (Port 1) - Algae States
    new JoystickButton(buttonBoard2, 1)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.ALGAE_STOWAGE),
            this
        ));

    new JoystickButton(buttonBoard2, 2)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.ALGAE_INTAKE),
            this
        ));

    new JoystickButton(buttonBoard2, 3)
        .onTrue(Commands.runOnce(
            () -> setState(SuperStructState.ALGAE_PLACEMENT),
            this
        ));

    // ... remaining buttons can be used for manual overrides if needed
  }

  public void L1() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(-0.1);
  }

  public void L2() {
    // mElevator.setPosition(0);
    mGrabber.setAngle(-0.045);
  }

  public void L3() {
    // mElevator.setPosition(0);
    mGrabber.setAngle(-0.99);
  }

  public void L4() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
  }

  public void TRAVEL() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    // mIntake.setSpeed(0.01);
  }

  public void CS() {
    mElevator.setPosition(-2.27534);
    mGrabber.setAngle(-0.18635);
    // mIntake.intake();
  }

  public void PLACEMENT() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    // mIntake.setSpeed(0.3);
  }

  public void DEFAULT() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    // mIntake.setSpeed(0);
    mAlgaeIntake.setIntakeSpeed(0);
    mAlgaeIntake.setAngle(-0.403076);
  }

  public void ALGAE_STOWAGE() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    mAlgaeIntake.setAngle(-0.433076);
    mAlgaeIntake.setIntakeSpeed(-0.0);
  }

  public void ALGAE_INTAKE() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    mAlgaeIntake.setAngle(-0.271973);
    mAlgaeIntake.intake();
  }

  public void ALGAE_PLACEMENT() {
    // mElevator.setPosition(0);
    // mGrabber.setAngle(0);
    mAlgaeIntake.setAngle(-0.403076);
    mAlgaeIntake.setIntakeSpeed(0.6);
  }

  public void updateState() {
    switch (mCommandedState) {
        case L1:
            L1();
            break;
        case L2:
            L2();
            break;
        case L3:
            L3();
            break;
        case L4:
            L4();
            break;
        case TRAVEL:
            TRAVEL();
            break;
        case CS:
            CS();
            break;
        case PLACEMENT:
            PLACEMENT();
            break;
        case DEFAULT:
            DEFAULT();
            break;
        case ALGAE_STOWAGE:
            ALGAE_STOWAGE();
            break;
        case ALGAE_INTAKE:
            ALGAE_INTAKE();
            break;
        case ALGAE_PLACEMENT:
            ALGAE_PLACEMENT();
            break;
    }
  }

  public void setState(SuperStructState state) {
    mStateMachine.setCommandedState(state);
  }
    
  /**
   * Handle Xbox controller inputs for state control (✿◠‿◠)
   */
  public void handleControllerInput() {
    // A button - L1
    if (controller.getAButton()) {
      setState(SuperStructState.L1);
    }
    // B button - L2
    else if (controller.getBButton()) {
      setState(SuperStructState.L2);
    }
    // Y button - L3
    else if (controller.getYButton()) {
      setState(SuperStructState.L3);
    }
    // X button - L4
    else if (controller.getXButton()) {
      setState(SuperStructState.L4);
    }
    // Right bumper - TRAVEL
    else if (controller.getRightBumperButton()) {
      setState(SuperStructState.TRAVEL);
    }
    // Left bumper - CS (CORAL Station)
    else if (controller.getLeftBumperButton()) {
      setState(SuperStructState.CS);
    }
    // Start button - PLACEMENT
    else if (controller.getStartButton()) {
      setState(SuperStructState.PLACEMENT);
    }
    // Back button - DEFAULT
    else if (controller.getBackButton()) {
      setState(SuperStructState.DEFAULT);
    }
    // POV Up - ALGAE_STOWAGE
    else if (controller.getPOV() == 0) {
      setState(SuperStructState.ALGAE_STOWAGE);
    }
    // POV Right - ALGAE_INTAKE
    else if (controller.getPOV() == 90) {
      setState(SuperStructState.ALGAE_INTAKE);
    }
    // POV Down - ALGAE_PLACEMENT
    else if (controller.getPOV() == 180) {
      setState(SuperStructState.ALGAE_PLACEMENT);
    }
  }

  @Override
  public void periodic() {
    // Handle controller input first
    // handleControllerInput();
    
    // This method will be called once per scheduler run
    mCommandedState = mStateMachine.getCommandedState();
    updateState();

    SmartDashboard.putString("Commanded State", mCommandedState.toString());
    
    // Add button state monitoring to SmartDashboard
    SmartDashboard.putBoolean("Board 1 Button 1 (L1)", buttonBoard1.getRawButton(1));
    SmartDashboard.putBoolean("Board 1 Button 2 (L2)", buttonBoard1.getRawButton(2));
    // ... add more button states as needed ...
  }
}
