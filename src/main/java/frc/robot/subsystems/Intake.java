// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private final SparkMax up;
    private final SparkMax down;

    // Singleton instance
    private static Intake mInstance = null;

    // Speed constants
    private static final double INTAKE_SPEED = 0.25;    // Speed for intaking (negative for inward)
    private static final double OUTAKE_SPEED = -0.2;     // Speed for outaking (positive for outward)
    private static final double SPEED_RATIO = 1;      // Down motor runs at 80% of up motor speed

    // Stall detection constants
    private static final double STALL_RPM_THRESHOLD = 50.0;     // RPM below this is considered stalled
    private static final int STARTUP_CYCLES = 10;               // Cycles to wait for startup
    private static final int STALL_TIMEOUT = 50;               // Cycles before considering motor stuck
    
    // Stall detection variables
    private int startupCounter = 0;
    private int stallCounter = 0;

    // Shuffleboard entries
    private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    private final GenericEntry bothInButton = intakeTab.add("Both Motors In", false)
        .withWidget("Toggle Button")
        .withPosition(0, 5)
        .getEntry();
    private final GenericEntry bothOutButton = intakeTab.add("Both Motors Out", false)
        .withWidget("Toggle Button")
        .withPosition(1, 5)
        .getEntry();

    // Add RPM status indicators to Shuffleboard
    private final GenericEntry upRPMStatus = intakeTab.add("Up Motor OK", true)
        .withWidget("Boolean Box")
        .withPosition(0, 6)
        .getEntry();
    private final GenericEntry downRPMStatus = intakeTab.add("Down Motor OK", true)
        .withWidget("Boolean Box")
        .withPosition(1, 6)
        .getEntry();
    private final GenericEntry stallStatus = intakeTab.add("Stall Status", "Normal")
        .withPosition(2, 6)
        .getEntry();

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    private Intake() {
        // Initialize motors
        up = new SparkMax(18, MotorType.kBrushless);
        down = new SparkMax(19, MotorType.kBrushless);
        
        // Configure motors
        configureNEO(up, true);
        configureNEO(down, true);
    }

    private void configureNEO(SparkMax motor, boolean inverted) {
        SparkMaxConfig neoConfig = new SparkMaxConfig();
        neoConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .openLoopRampRate(0.1)
            .inverted(inverted);

        motor.setCANTimeout(250);
        motor.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intake() {
        double upRPM = Math.abs(up.getEncoder().getVelocity());
        double downRPM = Math.abs(down.getEncoder().getVelocity());
        
        // Add RPM monitoring to dashboard
        SmartDashboard.putNumber("Up Motor RPM", upRPM);
        SmartDashboard.putNumber("Down Motor RPM", downRPM);
        
        if (startupCounter < STARTUP_CYCLES) {
            // Initial startup phase
            up.set(INTAKE_SPEED);
            down.set(INTAKE_SPEED * SPEED_RATIO);
            startupCounter++;
            stallStatus.setString("Starting Up");
        } 
        else if (upRPM < STALL_RPM_THRESHOLD || downRPM < STALL_RPM_THRESHOLD) {
            // Potential stall detected
            if (stallCounter < STALL_TIMEOUT) {
                stallCounter++;
                up.set(INTAKE_SPEED);
                down.set(INTAKE_SPEED * SPEED_RATIO);
                stallStatus.setString("Possible Stall: " + stallCounter);
            } 
            else {
                // Confirmed stall - stop motors and reset
                stop();
                bothInButton.setBoolean(false);
                stallStatus.setString("Stall Detected - Stopped");
            }
        } 
        else {
            // Normal operation
            stallCounter = 0;
            up.set(INTAKE_SPEED);
            down.set(INTAKE_SPEED * SPEED_RATIO);
            stallStatus.setString("Running Normal");
        }
        
        // Update status indicators
        upRPMStatus.setBoolean(upRPM >= STALL_RPM_THRESHOLD);
        downRPMStatus.setBoolean(downRPM >= STALL_RPM_THRESHOLD);
    }

    @Override
    public void periodic() {
        // Handle synchronized movement
        if (bothOutButton.getBoolean(false)) {
            up.set(OUTAKE_SPEED);
            down.set(OUTAKE_SPEED * SPEED_RATIO);
            stallStatus.setString("Outaking");
        } else if (bothInButton.getBoolean(false)) {
            intake();  // Use stall detection for intake
        } else {
            stop();
            stallStatus.setString("Stopped");
        }

        // Update dashboard with motor speeds
        SmartDashboard.putNumber("Up Motor Speed", up.get());
        SmartDashboard.putNumber("Down Motor Speed", down.get());
    }

    public void stop() {
        up.set(0);
        down.set(0);
        startupCounter = 0;
        stallCounter = 0;
    }

    public void setSpeed(double speed) {
        up.set(speed);
        down.set(speed * SPEED_RATIO);
    }
}
