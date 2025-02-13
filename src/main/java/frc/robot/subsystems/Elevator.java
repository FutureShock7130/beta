// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import java.util.Map;
import frc.robot.states.SuperStructState;

public class Elevator extends SubsystemBase {
  // Motors
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  // Constants
  private static final double kG = -0.01; // Gravity feed forward constant -0.015
  private static final double kDownSpeedMultiplier = 0.5; // Reduces down speed

  // Singleton instance
  private static Elevator mInstance = null;

  public static synchronized Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  // PID Controller
  private final PIDController pidController = new PIDController(0.069, 0.00069, 0.0069);

  // State tracking
  private boolean positionLocked = false;
  private double targetPosition = 0.0;

  // Sensors
  private final CANcoder elevatorCoder;

  // Maximum positions tracking
  private double maxLeftRotations = 0.0;
  private double maxRightRotations = 0.0;

  // Shuffleboard entries
  private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
  private final GenericEntry upButton = elevatorTab.add("Elevator Up", false)
      .withWidget("Toggle Button")
      .withPosition(0, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry downButton = elevatorTab.add("Elevator Down", false)
      .withWidget("Toggle Button")
      .withPosition(1, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry L2Button = elevatorTab.add("Elevator L2", false)
      .withWidget("Toggle Button")
      .withPosition(2, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry L3Button = elevatorTab.add("Elevator L3", false)
      .withWidget("Toggle Button")
      .withPosition(3, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry L4Button = elevatorTab.add("Elevator L4", false)
      .withWidget("Toggle Button")
      .withPosition(4, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry GroundButton = elevatorTab.add("Elevator ground", false)
      .withWidget("Toggle Button")
      .withPosition(5, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry speedEntry = elevatorTab.add("Elevator Speed", 0.0)
      .withPosition(0, 1)
      .withSize(2, 1)
      .getEntry();

  private final GenericEntry positionEntry = elevatorTab.add("Elevator Position", 0.0)
      .withPosition(0, 2)
      .withSize(2, 1)
      .getEntry();

  private final GenericEntry maxLeftRotationsEntry = elevatorTab.add("Max Left Motor Rotations", 0.0)
      .withPosition(0, 5)
      .withSize(2, 1)
      .getEntry();

  private final GenericEntry maxRightRotationsEntry = elevatorTab.add("Max Right Motor Rotations", 0.0)
      .withPosition(0, 6)
      .withSize(2, 1)
      .getEntry();

  private final GenericEntry lockButton = elevatorTab.add("Lock Position", false)
      .withWidget("Toggle Button")
      .withPosition(2, 0)
      .withSize(1, 1)
      .getEntry();

  private final GenericEntry absolutePositionEntry = elevatorTab.add("Elevator Absolute Position", 0.0)
      .withPosition(0, 7)
      .withSize(2, 1)
      .getEntry();

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    leftMotor = new SparkMax(28, MotorType.kBrushless); // Update ID as needed
    rightMotor = new SparkMax(27, MotorType.kBrushless); // Update ID as needed

    configureNEO(leftMotor, false, true); // Invert both motors to make default direction clockwise
    configureNEO(rightMotor, false, true); // Both motors still turn the same way

    // // Initialize CANcoder
    elevatorCoder = new CANcoder(7);
    configureCANcoder();
  }

  private void configureNEO(SparkMax motor, boolean inverted, boolean softlimit) {
    SparkMaxConfig neoConfig = new SparkMaxConfig();

    // Create soft limit config for elevator
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(0) // Adjust these limits for your elevator!
        .forwardSoftLimitEnabled(softlimit)
        .reverseSoftLimit(-110) // Bottom position
        .reverseSoftLimitEnabled(softlimit);

    neoConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake) // Use brake mode for elevator
        .voltageCompensation(12.0)
        .openLoopRampRate(0.1)
        .apply(softLimitConfig)
        .inverted(inverted);

    // Add follow configuration for right motor
    if (motor == rightMotor) {
      neoConfig.follow(leftMotor);
    }

    motor.setCANTimeout(250);
    motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(0.0); // Reset encoder to zero
  }

  /**
   * Configure the CANcoder for absolute position reading (◕ᴗ◕✿)
   */
  private void configureCANcoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Configure for absolute position mode
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // config.MagnetSensor.MagnetOffset = 0.0;

    // Apply configuration
    elevatorCoder.getConfigurator().apply(config);
    elevatorCoder.setPosition(0);
    // Wait for config to apply
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * Get the absolute position from CANcoder! (✿◠‿◠)
   */
  public double getPosition() {
    return elevatorCoder.getPosition().getValueAsDouble();
  }

  public void setPosition(double targetPosition) {
    // Update PID setpoint
    pidController.setSetpoint(targetPosition);

    // Calculate PID output
    // double currentPosition = leftMotor.getEncoder().getPosition();
    double currentPosition = elevatorCoder.getPosition().getValueAsDouble();
    double pidOutput = pidController.calculate(currentPosition);

    // Add gravity compensation
    double gravityCompensation = (pidOutput >= 0) ? kG : 0.0;

    // Set motor output
    setElevatorSpeed(pidOutput);
  }

  public boolean isAtPosition() {
    double currentPosition = leftMotor.getEncoder().getPosition();
    return Math.abs(currentPosition - pidController.getSetpoint()) < 0.1;
  }

  /**
   * Reset the NEO encoders based on CANcoder position ʕ•ᴥ•ʔ
   */
  public void resetToAbsolute() {
    double absolutePosition = getPosition();
    leftMotor.getEncoder().setPosition(absolutePosition);
    rightMotor.getEncoder().setPosition(absolutePosition);
  }

  /**
   * Sets the elevator speed. Positive values move up, negative values move down.
   * Includes gravity compensation when moving up and speed reduction when moving
   * down! ^w^
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setElevatorSpeed(double speed) {
    // Add gravity feedforward when moving up
    double gravityCompensation = (speed <= 0) ? kG : 0.0;

    // Reduce speed when moving down
    if (speed > 0) {
      speed *= kDownSpeedMultiplier;
    }

    leftMotor.set(speed + gravityCompensation); // Right motor follows automatically uwu
  }

  /**
   * Move the elevator up at a fixed speed
   */
  public void down() {
    setElevatorSpeed(0.3 + kG); // Adjust this value based on your needs!
  }

  /**
   * Move the elevator down at a fixed speed
   */
  public void up() {
    setElevatorSpeed(-0.3 + kG); // Adjust this value based on your needs!
  }

  /**
   * Stop the elevator
   */
  public void stop() {
    setElevatorSpeed(0.0 + kG);
  }

  public void lockPosition() {
    positionLocked = true;
    // targetPosition = encoder.getPosition().getValueAsDouble(); // cancoder
    targetPosition = leftMotor.getEncoder().getPosition(); // encoder
  }

  public void unlockPosition() {
    positionLocked = false;
  }

  @Override
  public void periodic() {
    // Handle position locking
    if (lockButton.getBoolean(false)) {
      if (!positionLocked) {
        lockPosition();
      }
      // Use CANcoder for position feedback
      double currentPosition = elevatorCoder.getPosition().getValueAsDouble();
      double output = pidController.calculate(currentPosition, targetPosition);

      // Add gravity compensation
      double gravityCompensation = (output >= 0) ? kG : 0.0;
      setElevatorSpeed(output + gravityCompensation);
    } else {
      if (positionLocked) {
        unlockPosition();
      }
      // Normal button control
      if (upButton.getBoolean(false)) {
        up();
      } else if (downButton.getBoolean(false)) {
        down();
      } else if (L2Button.getBoolean(false)) {
        setPosition(-3.0091015625);
      } else if (L3Button.getBoolean(false)) {
        setPosition(-4.44677734375);
      } else if (L4Button.getBoolean(false)) {
        setPosition(-4.8359375);
      } else if (GroundButton.getBoolean(false)) {
        setPosition(0);
      } else {
        stop();
      }
    }

    // Track maximum rotations
    double leftRotations = Math.abs(leftMotor.getEncoder().getPosition());
    double rightRotations = Math.abs(rightMotor.getEncoder().getPosition());

    maxLeftRotations = Math.max(maxLeftRotations, leftRotations);
    maxRightRotations = Math.max(maxRightRotations, rightRotations);

    // Update max rotation displays
    maxLeftRotationsEntry.setDouble(maxLeftRotations);
    maxRightRotationsEntry.setDouble(maxRightRotations);

    double currentHeight = elevatorCoder.getPosition().getValueAsDouble();
    double pidOutput = pidController.calculate(currentHeight);

    // Add gravity compensation!
    double gravityCompensation = kG;

    // setElevatorSpeed(pidOutput + gravityCompensation);

    // Update absolute position display
    // absolutePositionEntry.setDouble(getPosition());
    SmartDashboard.putNumber("pid output", pidOutput);
    positionEntry.setDouble(elevatorCoder.getPosition().getValueAsDouble());
  }
}
