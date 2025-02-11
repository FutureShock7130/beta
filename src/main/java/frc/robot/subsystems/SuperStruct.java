// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.StateMachine;
import frc.robot.states.SuperStructState;
import edu.wpi.first.wpilibj.XboxController;

public class SuperStruct extends SubsystemBase {
  Elevator mElevator;
  Grabber mGrabber;
  Intake mIntake;
  AlgaeIntake mAlgaeIntake;
  StateMachine mStateMachine;
  SuperStructState mCommandedState;
  private final XboxController controller;
  private static final int CONTROLLER_PORT = 0; // Adjust port as needed! uwu

  private static SuperStruct mInstance = null;

  public static SuperStruct getInstance() {
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
  }

  public void L1() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
  }

  public void L2() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
  }

  public void L3() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
  }

  public void L4() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
  }

  public void TRAVEL() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mIntake.setSpeed(0.01);
  }

  public void CS() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mIntake.intake();
  }

  public void PLACEMENT() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mIntake.setSpeed(0.3);
  }

  public void DEFAULT() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mIntake.setSpeed(0);
    mAlgaeIntake.setIntakeSpeed(0);
    mAlgaeIntake.setAngle(0);
  }

  public void ALGAE_STOWAGE() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mAlgaeIntake.setAngle(0);
  }

  public void ALGAE_INTAKE() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mAlgaeIntake.intake();
  }

  public void ALGAE_PLACEMENT() {
    mElevator.setPosition(0);
    mGrabber.setAngle(0);
    mAlgaeIntake.setAngle(0);
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
    // Left bumper - CS (Charging Station)
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
    handleControllerInput();
    
    // This method will be called once per scheduler run
    mCommandedState = mStateMachine.getCommandedState();
    updateState();

    SmartDashboard.putString("Commanded State", mCommandedState.toString());
    // Add controller info to dashboard
    SmartDashboard.putString("Last Button Pressed", getLastButtonPressed());
  }

  /**
   * Helper to get last button pressed for dashboard uwu
   */
  private String getLastButtonPressed() {
    if (controller.getAButton()) return "A - L1";
    if (controller.getBButton()) return "B - L2";
    if (controller.getYButton()) return "Y - L3";
    if (controller.getXButton()) return "X - L4";
    if (controller.getRightBumperButton()) return "RB - TRAVEL";
    if (controller.getLeftBumperButton()) return "LB - CS";
    if (controller.getStartButton()) return "Start - PLACEMENT";
    if (controller.getBackButton()) return "Back - DEFAULT";
    if (controller.getPOV() == 0) return "POV Up - ALGAE_STOWAGE";
    if (controller.getPOV() == 90) return "POV Right - ALGAE_INTAKE";
    if (controller.getPOV() == 180) return "POV Down - ALGAE_PLACEMENT";
    return "None";
  }
}
