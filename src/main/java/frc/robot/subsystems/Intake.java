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
    private int startupCounter = 0;
    private int stallCounter = 0;

    private static Intake mInstance = null;

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    // Shuffleboard entries
    private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    private final GenericEntry upButton = intakeTab.add("Up Motor", false)
        .withWidget("Toggle Button")
        .withPosition(0, 0)
        .getEntry();
    private final GenericEntry downButton = intakeTab.add("Down Motor", false)
        .withWidget("Toggle Button")
        .withPosition(0, 1)
        .getEntry();
    // private final GenericEntry upSpeed = intakeTab.add("Up Motor Speed", 0.5)
    //     .withWidget("Number Slider")
    //     .withProperties(Map.of("min", -1.0, "max", 1.0))
    //     .withPosition(1, 0)
    //     .getEntry();
    // private final GenericEntry downSpeed = intakeTab.add("Down Motor Speed", 0.5)
    //     .withWidget("Number Slider")
    //     .withProperties(Map.of("min", -1.0, "max", 1.0))
    //     .withPosition(1, 1)
    //     .getEntry();
    private final GenericEntry bothInButton = intakeTab.add("Both Motors In", false)
        .withWidget("Toggle Button")
        .withPosition(0, 5)
        .getEntry();
    private final GenericEntry bothOutButton = intakeTab.add("Both Motors Out", false)
        .withWidget("Toggle Button")
        .withPosition(1, 5)
        .getEntry();
    private final GenericEntry upRPMStatus = intakeTab.add("Up Motor RPM OK", true)
        .withWidget("Boolean Box")
        .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
        .withPosition(0, 6)
        .getEntry();
    private final GenericEntry downRPMStatus = intakeTab.add("Down Motor RPM OK", true)
        .withWidget("Boolean Box")
        .withProperties(Map.of("colorWhenTrue", "Lime", "colorWhenFalse", "Red"))
        .withPosition(1, 6)
        .getEntry();

    public Intake() {
        up = new SparkMax(18, MotorType.kBrushless);
        down = new SparkMax(19, MotorType.kBrushless);
        
        configureNEO550(up);
        configureNEO550(down);
    }

    

    private void configureNEO550(SparkMax motor) {
        SparkMaxConfig neo550Config = new SparkMaxConfig();
        neo550Config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0);
            
        
        motor.setCANTimeout(250);
        motor.configure(neo550Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    
        // Manual control through Shuffleboard
        boolean upButtonState = upButton.getBoolean(false);
        boolean downButtonState = downButton.getBoolean(false);
        // double upSpeedValue = upSpeed.getDouble(0.5);
        // double downSpeedValue = downSpeed.getDouble(0.5);

        // if (upButtonState) up.set(upSpeedValue);
        // if (downButtonState) down.set(downSpeedValue);

        // Handle synchronized movement
        if (bothOutButton.getBoolean(false)) {
            up.set(0.5);
            down.set(0.3);
        } else if (bothInButton.getBoolean(false)) {
            intake();
            // up.set(-0.5);
            // down.set(-0.5);
        }
        // up.set(0.75);
        // down.set(0.5);

        // Update RPM status
        updateRPMStatus();
    }

    public void setSpeed(double speed) {
        up.set(speed);
        down.set(speed);
    }

    public void intake() {
        double upRPM = Math.abs(up.getEncoder().getVelocity());
        double downRPM = Math.abs(down.getEncoder().getVelocity());
        
        if (startupCounter < 10) {
            up.set(-0.6);
            down.set(-0.6);
            startupCounter++;
        } else if (upRPM < 50 || downRPM < 50) {
            if (stallCounter < 50) {
                stallCounter++;
                up.set(-0.6);
                down.set(-0.6);
            } else {
                up.set(-0.6);
                down.set(-0.6);
                bothOutButton.setBoolean(false);
                startupCounter = 0;
                stallCounter = 0;
            }
        } else {
            stallCounter = 0;
            up.set(-0.6);
            down.set(-0.6);
        }
    }

    public void stop(){
        up.set(0);
        down.set(0);
    }

    public void putL4(){
        up.set(0.3);
        down.set(0.15);
    }

    public void launchCoral(){
        up.set(0.3);
        down.set(0.3);
    }

    private void updateRPMStatus() {
        double upRPM = Math.abs(up.getEncoder().getVelocity());
        double downRPM = Math.abs(down.getEncoder().getVelocity());
        
        upRPMStatus.setBoolean(upRPM >= 50);
        downRPMStatus.setBoolean(downRPM >= 50);
        
        SmartDashboard.putNumber("Up Motor RPM", upRPM);
        SmartDashboard.putNumber("Down Motor RPM", downRPM);
    }
}
