package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;
import edu.wpi.first.math.controller.PIDController;

public class Grabber extends SubsystemBase {
    private final SparkMax up;
    private final SparkMax down;
    private final SparkMax angle;
    // private final CANCoder angleEncoder;
    private static final double DEFAULT_KG = 0.003;
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    // private final GenericEntry upButton, downButton, upSpeed, downSpeed;
    // private final GenericEntry forwardButton, backwardButton, flatButton;
    // private final GenericEntry angleDisplay, kGTuner;
    // private final GenericEntry bothInButton, bothOutButton;
    // private final GenericEntry upRPMStatus, downRPMStatus;

    // private final ShuffleboardTab motorTab = Shuffleboard.getTab("Motor Controls");
  private final GenericEntry upButton = grabberTab.add("Up Motor", false)
      .withWidget("Toggle Button")
      .withPosition(0, 0)
      .getEntry();
  private final GenericEntry downButton = grabberTab.add("Down Motor", false)
      .withWidget("Toggle Button")
      .withPosition(0, 1)
      .getEntry();
  private final GenericEntry upSpeed = grabberTab.add("Up Motor Speed", 0.5)
      .withWidget("Number Slider")
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .withPosition(1, 0)
      .getEntry();
  private final GenericEntry downSpeed = grabberTab.add("Down Motor Speed", 0.5)
      .withWidget("Number Slider")
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .withPosition(1, 1)
      .getEntry();

  // Add preset angle buttons to Shuffleboard 
  private final GenericEntry forwardButton = grabberTab.add("Turn 45 Forward", false)
      .withWidget("Toggle Button")
      .withPosition(0, 4)
      .withSize(1, 1)
      .getEntry();
  private final GenericEntry backwardButton = grabberTab.add("Turn 45 Back", false)
      .withWidget("Toggle Button")
      .withPosition(1, 4)
      .withSize(1, 1)
      .getEntry();

    private final GenericEntry flatButton = grabberTab.add("Flat", false)
      .withWidget("Toggle Button")
      .withPosition(2, 4)
      .withSize(1, 1)
      .getEntry();    

  // Add angle display widget 
  private final GenericEntry angleDisplay = grabberTab.add("Current Angle", 0.0)
      .withWidget("Text View")  // Shows number clearly UwU
      .withPosition(2, 4)
      .withSize(1, 1)
      .getEntry();

  // Add kG tuning slider ✨
  private final GenericEntry kGTuner = grabberTab.add("Gravity Compensation", DEFAULT_KG)
      .withWidget("Number Slider")
      .withProperties(Map.of("min", 0.0, "max", 0.2))  // Reasonable range
      .withPosition(3, 4)
      .withSize(1, 1)
      .getEntry();

  // Add buttons for synchronized movement ✨
  private final GenericEntry bothInButton = grabberTab.add("Both Motors In", false)
      .withWidget("Toggle Button")
      .withPosition(0, 5)
      .withSize(1, 1)
      .getEntry();
  private final GenericEntry bothOutButton = grabberTab.add("Both Motors Out", false)
      .withWidget("Toggle Button")
      .withPosition(1, 5)
      .withSize(1, 1)
      .getEntry();

  // Add RPM monitoring widgets ✨
  private final GenericEntry upRPMStatus = grabberTab.add("Up Motor RPM OK", true)
      .withWidget("Boolean Box")  // Shows green/red status
      .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
      .withPosition(0, 6)
      .withSize(1, 1)
      .getEntry();
      
  private final GenericEntry downRPMStatus = grabberTab.add("Down Motor RPM OK", true)
      .withWidget("Boolean Box")
      .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
      .withPosition(1, 6)
      .withSize(1, 1)
      .getEntry();

  private final PIDController pidController = new PIDController(
      0.07,   // kP
      0.035,  // kI - helps eliminate steady-state error UwU
      0.007   // kD - reduces overshoot and oscillation ✨
  );

    public Grabber() {
        up = new SparkMax(2, MotorType.kBrushless);
        down = new SparkMax(3, MotorType.kBrushless);
        angle = new SparkMax(1, MotorType.kBrushless);
        // angleEncoder = angle.getEncoder();

        configureNEO550(up);
        configureNEO550(down);
        configureNEO(angle);
        
        angle.getEncoder().setPosition(0.0);

       
    }

    @Override
    public void periodic() {
        double kG = kGTuner.getDouble(DEFAULT_KG);
    
    if (forwardButton.getBoolean(false)) {
      angle.set(0.07 + kG);  // Now positive is forward
      if (angle.getEncoder().getPosition() >= 2.76) {  // For 53° at 18.75:1
        angle.set(kG);
        
      }
      
    } else if (backwardButton.getBoolean(false)) {
      angle.set(-0.025);  // Now negative is backward
      if (angle.getEncoder().getPosition() <= 0) {
        angle.set(0);
        }
    } else if (flatButton.getBoolean(false)) {
      angle.set(0.07 + kG);  // Move forward
      if (angle.getEncoder().getPosition() >= 1.71875) {  // 33° position
        angle.set(kG);
        }
    } else {
      angle.set(kG * 0.5);  // Flipped kG direction too!
    }

    // Get button states and speeds fow motow contwol OwO
    boolean upButtonState = upButton.getBoolean(false);
    boolean downButtonState = downButton.getBoolean(false);
    
    double upSpeedValue = upSpeed.getDouble(0.5);
    double downSpeedValue = downSpeed.getDouble(0.5);
    
    up.set(upButtonState ? upSpeedValue : 0);
    down.set(downButtonState ? downSpeedValue : 0);

    // Update angle display with new ratio
    double currentAngleDegrees = angle.getEncoder().getPosition() * (360.0 / 18.75);  // Convert from rotations
    angleDisplay.setDouble(currentAngleDegrees);

    // Handle synchronized motor control
    if (bothInButton.getBoolean(false)) {
      up.set(0.3);    // Up motor forward
      down.set(0.15); // Down motor reverse
    } else if (bothOutButton.getBoolean(false)) {
      double upRPM = Math.abs(up.getEncoder().getVelocity());
      double downRPM = Math.abs(down.getEncoder().getVelocity());
      
      if (startupCounter < 10) {  // Startup delay
        up.set(-0.4);
        down.set(-0.4);
        startupCounter++;
      } else if (upRPM < 50 || downRPM < 50) {
        if (stallCounter < 50) {  // Wait ~0.5 seconds (25 * 20ms) before stopping
          stallCounter++;
          up.set(-0.4);
          down.set(-0.4);
        } else {
          up.set(0);
          down.set(0);
          bothOutButton.setBoolean(false);
          startupCounter = 0;
          stallCounter = 0;
        }
      } else {
        stallCounter = 0;  // Reset stall counter if RPM is good
        up.set(-0.4);
        down.set(-0.4);
      }
    } else {
      startupCounter = 0;  // Reset both counters when button released
      stallCounter = 0;
    }

    // Check RPM and update status
    double upRPM = Math.abs(up.getEncoder().getVelocity());
    double downRPM = Math.abs(down.getEncoder().getVelocity());
    
    SmartDashboard.putNumber("rpm", up.getEncoder().getVelocity());
        // ... (rest of your control logic)
    }

    // Copy your configuration methods
    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neo550Config = new SparkMaxConfig();
    
        neo550Config
            .smartCurrentLimit(40)  // Protect our smol motors! >w<
            .idleMode(IdleMode.kCoast)  // Better control!
            .voltageCompensation(12.0)  // Stable performance! UwU
            .openLoopRampRate(0.1);     // Smooth acceleration! 
    
        // Apply our configuration with proper timeout
        motor.setCANTimeout(250);
        motor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureNEO(SparkMax motor) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();
    
    // Create soft limit config 
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimit(2.76)    // +53 degrees with 18.75:1
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0)       // Keep starting point
            .reverseSoftLimitEnabled(true);
    
        neoConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .openLoopRampRate(0.1)
            .apply(softLimitConfig)
            .inverted(false);   // Flips motor direction
    
        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
} 