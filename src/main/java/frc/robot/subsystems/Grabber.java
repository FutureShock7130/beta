package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.jni.CANCommonJNI;
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
    private final SparkMax angle;
    private static final double kG = -0.02;
    private final CANcoder cancoder;

    private static Grabber mInstance = null;

    public static synchronized Grabber getInstance() {
        if (mInstance == null) {
            mInstance = new Grabber();
        }
        return mInstance;
    }

    // Shuffleboard entries for angle control only
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    private final GenericEntry angleDisplay;
    private final PIDController pidController;

    // Add to your existing Shuffleboard entries
    private final GenericEntry upButton = grabberTab.add("Grabber Up", false)
        .withWidget("Toggle Button")
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();
        
    private final GenericEntry downButton = grabberTab.add("Grabber Down", false)
        .withWidget("Toggle Button")
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();
        
    private final GenericEntry stopButton = grabberTab.add("Grabber Stop", false)
        .withWidget("Toggle Button")
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
        
    private final GenericEntry speedSlider = grabberTab.add("Manual Speed", 0.05)
        .withWidget("Number Slider")
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .withPosition(0, 1)
        .withSize(3, 1)
        .getEntry();

    private final GenericEntry CSButton = grabberTab.add("cs", false)
    .withWidget("Toggle Button")
    .withProperties(Map.of("min", -1.0, "max", 1.0))
    .withPosition(0, 2)
    .withSize(3, 1)
    .getEntry();

    private final GenericEntry L2Button = grabberTab.add("l2", false)
    .withWidget("Toggle Button")
    .withProperties(Map.of("min", -1.0, "max", 1.0))
    .withPosition(0, 3)
    .withSize(3, 1)
    .getEntry();

    private final GenericEntry L3Button = grabberTab.add("l3", false)
    .withWidget("Toggle Button")
    .withProperties(Map.of("min", -1.0, "max", 1.0))
    .withPosition(0, 4)
    .withSize(3, 1)
    .getEntry();
    // Add these variables
    private double maxRotations = 0.0;

    
    
    // Add to your Shuffleboard entries
    private final GenericEntry maxRotationsEntry = grabberTab.add("Max Motor Rotations", 0.0)
        .withPosition(0, 5)
        .withSize(2, 1)
        .getEntry();
        
    private final GenericEntry resetMaxButton = grabberTab.add("Reset Max", false)
        .withWidget("Toggle Button")
        .withPosition(2, 5)
        .withSize(1, 1)
        .getEntry();

    public Grabber() {
        angle = new SparkMax(17, MotorType.kBrushless);
        configureNEO(angle, true, true);
        angle.getEncoder().setPosition(0.0);
        cancoder = new CANcoder(4);
        configureCANcoder();

        // Initialize Shuffleboard entries
        angleDisplay = grabberTab.add("Current Angle", 0.0)
                .withWidget("Text View")
                .withPosition(2, 4)
                .getEntry();


        pidController = new PIDController(0.4, 0.004, 0.02);
        
    }

    public void setSpeed(double speed) {
        // Add gravity feedforward when moving up
        double gravityCompensation = (speed >= 0) ? kG : 0.0;

        angle.set(speed + gravityCompensation);
    }

    /**
     * Sets the grabber to a specific angle (✿◠‿◠)
     * 
     * @param targetAngleDegrees The desired angle in degrees
     */
    public void setAngle(double targetRotations) {
        // Convert degrees to motor rotations (18.75:1 gear ratio)
        // double targetRotations = targetAngleDegrees * (18.75 / 360.0); // encoder

        // Update PID setpoint
        pidController.setSetpoint(targetRotations);

        setSpeed(pidController.calculate(cancoder.getAbsolutePosition().getValueAsDouble()));
    }

    /**
     * Checks if the grabber is at the target angle (◕ᴗ◕✿)
     * 
     * @return true if at target angle, false otherwise
     */
    public boolean isAtTargetAngle() {
        // double currentAngle = angle.getEncoder().getPosition(); //encoder
        double currentAngle = cancoder.getAbsolutePosition().getValueAsDouble();
        return Math.abs(currentAngle - pidController.getSetpoint()) < 0.1;
    }

    @Override
    public void periodic() {
        double currentAngle = cancoder.getAbsolutePosition().getValueAsDouble();
        // double kG = kGTuner.getDouble(0.003);

        // Use PID to maintain target angle
        // angle.set(pidController.calculate(currentAngle) + kG);

        // Handle manual control buttons
        if (upButton.getBoolean(false)) {
            // double speed = speedSlider.getDouble(0.05);
            setSpeed(-0.05);
            // setAngle(-0.1);
        } 
        else if (downButton.getBoolean(false)) {
            // double speed = -speedSlider.getDouble(0.3);
            setSpeed(0.02);
        }
        else 
        // if (stopButton.getBoolean(false)) {
        //     setSpeed(0);
        // } else
         if (CSButton.getBoolean(false)) {
            setAngle(0.19);
        }
        else if (L2Button.getBoolean(false)) {
            setAngle(0.22);
        }
        else if (L3Button.getBoolean(false)) {
            setAngle(0.235);
        }
        else {
            setSpeed(0);
        }

        // Track maximum rotations
        double currentRotations = angle.getEncoder().getPosition();
        if (Math.abs(currentRotations) > Math.abs(maxRotations)) {
            maxRotations = currentRotations;
            maxRotationsEntry.setDouble(maxRotations);
        }

        

        // Update displays
        double currentAngleDegrees = cancoder.getAbsolutePosition().getValueAsDouble() * (360.0 / 18.75);
        angleDisplay.setDouble(currentAngle);
        SmartDashboard.putNumber("pidoutput grab", pidController.calculate(currentAngle));
        SmartDashboard.putNumber("Grabber Current Angle", 
            cancoder.getAbsolutePosition().getValueAsDouble() );
        SmartDashboard.putNumber("Grabber Target Angle", 
            pidController.getSetpoint());
        SmartDashboard.putBoolean("At Target Angle", 
            isAtTargetAngle());
    }

    private void configureNEO(SparkMax motor, boolean inverted, boolean softlimit) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();

        // Create soft limit config
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
                .forwardSoftLimit(0) // +53 degrees with 18.75:1
                .forwardSoftLimitEnabled(softlimit)
                .reverseSoftLimit(-3.15) // Keep starting point
                .reverseSoftLimitEnabled(softlimit);

        neoConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0)
                .openLoopRampRate(0.1)
                .apply(softLimitConfig)
                .inverted(inverted); // Flips motor direction

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configure the CANcoder for absolute position reading (◕ᴗ◕✿)
     */
    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Apply configuration
        cancoder.getConfigurator().apply(config);

        // Wait for config to apply
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}