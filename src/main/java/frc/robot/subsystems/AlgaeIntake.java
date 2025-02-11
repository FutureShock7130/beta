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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax leftAngle;
  private final SparkMax rightAngle;
  private final CANcoder angleEncoder;
  private final SparkMax intakeMotor;
  private final PIDController anglePIDController;
  private static final double ANGLE_kP = 0.01;    // Adjust these PID values! uwu
  private static final double ANGLE_kI = 0.0;
  private static final double ANGLE_kD = 0.0;
  private static final double ANGLE_TOLERANCE = 2.0; // Degrees

  private static AlgaeIntake mInstance = null;
    
  public static AlgaeIntake getInstance() {
      if (mInstance == null) {
          mInstance = new AlgaeIntake();
      }
      return mInstance;
  }

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    leftAngle = new SparkMax(10, MotorType.kBrushless);
    rightAngle = new SparkMax(11, MotorType.kBrushless);
    angleEncoder = new CANcoder(12, "GTX7130");
    intakeMotor = new SparkMax(13, MotorType.kBrushless);

    configureNEO(leftAngle, false, true);
    configureSlaveNEO(rightAngle, false, true);
    configureNEO(intakeMotor, false, false);
    configureCANcoder();

    // Initialize angle PID controller
    anglePIDController = new PIDController(ANGLE_kP, ANGLE_kI, ANGLE_kD);
    anglePIDController.setTolerance(ANGLE_TOLERANCE);
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
        .forwardSoftLimit(0.0)    //45:1 gear reduction
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

  private void configureCANcoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    
    // Configure for absolute position mode
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    
    // Apply configuration
    angleEncoder.getConfigurator().apply(config);
    
  }

  /**
   * Sets the angle of the intake using CANcoder feedback (✿◠‿◠)
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

  /**
   * Checks if intake is at the target angle (◕ᴗ◕✿)
   * @return true if at target angle, false otherwise
   */
  public boolean isAtAngle() {
    return anglePIDController.atSetpoint();
  }

  /**
   * Gets the current angle from CANcoder uwu
   * @return current angle in degrees
   */
  public double getCurrentAngle() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public void intake() {
    intakeMotor.set(0.2);
  }

  public void outgae() {     //get it?
    intakeMotor.set(-0.2);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Add angle info to dashboard
    SmartDashboard.putNumber("Algae Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("Algae Target Angle", anglePIDController.getSetpoint());
    SmartDashboard.putBoolean("Algae At Target Angle", isAtAngle());
  }
}
