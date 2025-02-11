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
    private final SparkMax angle;
    private static final double DEFAULT_KG = 0.003;

    private static Grabber mInstance = null;

    public static Grabber getInstance() {
        if (mInstance == null) {
            mInstance = new Grabber();
        }
        return mInstance;
    }

    // Shuffleboard entries for angle control only
    private final ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    private final GenericEntry angleDisplay;
    private final GenericEntry kGTuner;
    private final GenericEntry stateDisplay;
    private final PIDController pidController;

    public Grabber() {
        angle = new SparkMax(1, MotorType.kBrushless);
        configureNEO(angle, false, true);
        angle.getEncoder().setPosition(0.0);

        // Initialize Shuffleboard entries
        angleDisplay = grabberTab.add("Current Angle", 0.0)
            .withWidget("Text View")
            .withPosition(2, 4)
            .getEntry();

        kGTuner = grabberTab.add("Gravity Compensation", DEFAULT_KG)
            .withWidget("Number Slider")
            .withProperties(Map.of("min", 0.0, "max", 0.2))
            .withPosition(3, 4)
            .getEntry();

        stateDisplay = grabberTab.add("Current State", "DEFAULT")
            .withWidget("Text View")
            .withPosition(2, 0)
            .getEntry();

        pidController = new PIDController(0.07, 0.035, 0.007);
    }

    /**
     * Sets the grabber to a specific angle (✿◠‿◠)
     * @param targetAngleDegrees The desired angle in degrees
     */
    public void setAngle(double targetAngleDegrees) {
        // Convert degrees to motor rotations (18.75:1 gear ratio)
        double targetRotations = targetAngleDegrees * (18.75 / 360.0);
        
        // Update PID setpoint
        pidController.setSetpoint(targetRotations);
    }

    /**
     * Checks if the grabber is at the target angle (◕ᴗ◕✿)
     * @return true if at target angle, false otherwise
     */
    public boolean isAtTargetAngle() {
        double currentAngle = angle.getEncoder().getPosition();
        return Math.abs(currentAngle - pidController.getSetpoint()) < 0.1;
    }

    @Override
    public void periodic() {
        double currentAngle = angle.getEncoder().getPosition();
        double kG = kGTuner.getDouble(DEFAULT_KG);

        // Use PID to maintain target angle
        angle.set(pidController.calculate(currentAngle) + kG);

        // Update displays
        double currentAngleDegrees = angle.getEncoder().getPosition() * (360.0 / 18.75);
        angleDisplay.setDouble(currentAngleDegrees);
    }

    private void configureNEO(SparkMax motor, boolean inverted, boolean softlimit) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();
        
    // Create soft limit config 
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig
            .forwardSoftLimit(2.76)    // +53 degrees with 18.75:1
            .forwardSoftLimitEnabled(softlimit)
            .reverseSoftLimit(0)       // Keep starting point
            .reverseSoftLimitEnabled(softlimit);
    
        neoConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .openLoopRampRate(0.1)
            .apply(softLimitConfig)
            .inverted(inverted);   // Flips motor direction
        
        
        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
} 