// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax leftAngle;
  private final SparkMax rightAngle;
  private final CANcoder angleEncoder;
  private final SparkMax intakeMotor;
  private final SparkMax duckMotor;
  private final PIDController anglePIDController;
  private final PIDController climbPIDController;
  private static final double ANGLE_kP = 1.69; // Adjust these PID values! uwu
  private static final double ANGLE_kI = 0.001;
  private static final double ANGLE_kD = 0.0069;
  private static final double ANGLE_TOLERANCE = 5.0; // Degrees
  private static final double CLIMB_KP = 10;
  private static final double CLIMB_KI = 1;
  private static final double CLIMB_KD = 0.5;
  
  private int startupCounter = 0;
  private int stallCounter = 0;

  // private final ShuffleboardTab algaeIntakeTab = Shuffleboard.getTab("AlgaeInatke");
  // private final GenericEntry angleDisplay;
  // private final GenericEntry duckUpButton;
  // private final GenericEntry duckDownButton;
  // private final GenericEntry duckStopButton;

  private static AlgaeIntake mInstance = null;

  public static synchronized AlgaeIntake getInstance() {
    if (mInstance == null) {
      mInstance = new AlgaeIntake();
    }
    return mInstance;
  }

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    leftAngle = new SparkMax(38, MotorType.kBrushless);
    rightAngle = new SparkMax(37, MotorType.kBrushless);
    angleEncoder = new CANcoder(5, "GTX7130");
    intakeMotor = new SparkMax(36, MotorType.kBrushless);
    duckMotor = new SparkMax(39, MotorType.kBrushless);

    configureNEO(leftAngle, true, false);
    configureSlaveNEO(rightAngle, false, false);
    configureNEO550(intakeMotor, false, false);
    configureNEO550(duckMotor, false, false);
    configureCANcoder();

    // Initialize angle PID controller
    anglePIDController = new PIDController(ANGLE_kP, ANGLE_kI, ANGLE_kD);
    anglePIDController.setTolerance(ANGLE_TOLERANCE);
    climbPIDController = new PIDController(CLIMB_KP, CLIMB_KI, CLIMB_KD);

    // angleDisplay = algaeIntakeTab.add("Current Angle", 0.0)
    //     .withWidget(BuiltInWidgets.kTextView)
    //     .withPosition(2, 4)
    //     .getEntry();

    // duckUpButton = algaeIntakeTab.add("Duck Up", false)
    //     .withWidget("Toggle Button")
    //     .withPosition(0, 0)
    //     .withSize(1, 1)
    //     .getEntry();
        
    // duckDownButton = algaeIntakeTab.add("Duck Down", false)
    //     .withWidget("Toggle Button")
    //     .withPosition(1, 0)
    //     .withSize(1, 1)
    //     .getEntry();
        
    // duckStopButton = algaeIntakeTab.add("Duck Stop", false)
    //     .withWidget("Toggle Button")
    //     .withPosition(2, 0)
    //     .withSize(1, 1)
    //     .getEntry();
  }

  public void configureNEO(SparkMax motor, boolean inverted, boolean softlimit) {
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(0.0)
        .forwardSoftLimitEnabled(softlimit)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(softlimit);

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .inverted(inverted)
        .apply(softLimitConfig);

    motor.setCANTimeout(250);
    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configureSlaveNEO(SparkMax motor, boolean inverted, boolean softlimit) {
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(0.0) // 45:1 gear reduction
        .forwardSoftLimitEnabled(softlimit)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(softlimit);

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .inverted(inverted)
        .apply(softLimitConfig)
        .follow(10);

    motor.setCANTimeout(250);
    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configureNEO550(SparkMax motor, boolean inverted, boolean softlimit) {
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig
        .forwardSoftLimit(0.0) // 45:1 gear reduction
        .forwardSoftLimitEnabled(softlimit)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(softlimit);

    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .inverted(inverted)
        .apply(softLimitConfig);

    motor.setCANTimeout(250);
    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureCANcoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Apply configuration
    angleEncoder.getConfigurator().apply(config);

  }

  /**
   * Sets the angle of the intake using CANcoder feedback (✿◠‿◠)
   * 
   * @param targetAngleDegrees The desired angle in degrees
   */
  public void setAngle(double targetAngleDegrees) {
    // Update PID setpoint
    anglePIDController.setSetpoint(targetAngleDegrees);

    // Get current angle from CANcoder
    double currentAngle = angleEncoder.getAbsolutePosition().getValueAsDouble();

    // Calculate PID output
    double pidOutput = anglePIDController.calculate(currentAngle);

    // Limit output for safety ʕ•ᴥ•ʔ
    pidOutput = MathUtil.clamp(pidOutput, -0.4, 0.4);

    // Set motor outputs (both left and right angle motors)
    leftAngle.set(pidOutput);
  }

  public void setClimbAngle(double targetAngleDegrees) {
    // Update PID setpoint
    climbPIDController.setSetpoint(targetAngleDegrees);

    // Get current angle from CANcoder
    double currentAngle = angleEncoder.getAbsolutePosition().getValueAsDouble();

    // Calculate PID output
    double pidOutput = climbPIDController.calculate(currentAngle);

    // Limit output for safety ʕ•ᴥ•ʔ
    pidOutput = MathUtil.clamp(pidOutput, -0.8, 0.8);

    // Set motor outputs (both left and right angle motors)
    leftAngle.set(pidOutput);
  }

  /**
   * Checks if intake is at the target angle (◕ᴗ◕✿)
   * 
   * @return true if at target angle, false otherwise
   */
  public boolean isAtAngle() {
    return anglePIDController.atSetpoint();
  }

  /**
   * Gets the current angle from CANcoder uwu
   * 
   * @return current angle in degrees
   */
  public double getCurrentAngle() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public void intake() {
    intakeMotor.set(-0.7);

    double algaeIntakeRPM = Math.abs(intakeMotor.getEncoder().getVelocity());
  

    if (startupCounter < 10) {
      intakeMotor.set(-0.6);
      
      startupCounter++;
    } else if (algaeIntakeRPM < 50 ) {
      if (stallCounter < 50) {
        stallCounter++;
        intakeMotor.set(-0.6);
        
      } else {
        intakeMotor.set(-0.0);
        startupCounter = 0;
        stallCounter = 0;
      }
    } else {
      stallCounter = 0;
      intakeMotor.set(-0.6);
    }
  }

  public void outgae() { // get it?
    intakeMotor.set(0.7);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setDuckSpeed(double speed) {
    duckMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Handle manual control buttons
    //   if (duckUpButton.getBoolean(false)) {
    //     setDuckSpeed(0.3);
    // } 
    // else if (duckDownButton.getBoolean(false)) {
    //     setDuckSpeed(-0.3);
    // }
    // else if (duckStopButton.getBoolean(false)) {
    //     setDuckSpeed(0);
    // } else {
    //   setDuckSpeed(0);
    // }

    // Add angle info to dashboard
    SmartDashboard.putNumber("Algae Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("Algae Target Angle", anglePIDController.getSetpoint());
    SmartDashboard.putBoolean("Algae At Target Angle", isAtAngle());
  }
}
