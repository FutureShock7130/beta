package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.FSLib2025.chassis.OnboardModuleState;
import frc.FSLib2025.chassis.SwerveModuleConstants;
import frc.FSLib2025.control.Feedforward;
import frc.FSLib2025.control.PID;
import frc.FSLib2025.math.FS_Math;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public int ModuleNumber;
    public SwerveModuleConstants ModuleConstants;

    private SparkMax steerMotor;
    public TalonFX driveMotor;

    private CANcoder steerCANcoder;

    private PID steerPID;

    private Rotation2d lastAngle;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.ModuleNumber = moduleNumber;
        this.ModuleConstants = moduleConstants;

        steerPID = new PID(
                SwerveConstants.STEER_MOTOR_KP,
                SwerveConstants.STEER_MOTOR_KI,
                SwerveConstants.STEER_MOTOR_KD,
                SwerveConstants.STEER_MOTOR_WINDUP,
                SwerveConstants.STEER_MOTOR_LIMIT);

        lastAngle = new Rotation2d();

        steerMotor = new SparkMax(moduleConstants.SteerMotorId, MotorType.kBrushless);
        SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
        steerMotorConfig.inverted(false);
        steerMotor.setControlFramePeriodMs(20);
        steerMotorConfig.idleMode(IdleMode.kBrake);
        steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        steerCANcoder = new CANcoder(moduleConstants.CANcoderId, "GTX7130");
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        steerCANcoder.getConfigurator().apply(cancoderConfig);

        driveMotor = new TalonFX(moduleConstants.DriveMotorId, "GTX7130");
        TalonFXConfiguration talonfxConfig = new TalonFXConfiguration();
        talonfxConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_MOTOR_GEAR_RATIO;
        driveMotor.getConfigurator().apply(talonfxConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = OnboardModuleState.optimize(desiredState, getModuleState().angle);

        desiredState.angle = Math.abs(desiredState.speedMetersPerSecond) < 0.01 ? lastAngle : desiredState.angle;
        double error = getModuleState().angle.getDegrees() - desiredState.angle.getDegrees();
        error = FS_Math.constrainAngleDegrees(error);
        double steerOutput = steerPID.calculate(error);
        setSteerMotor(steerOutput);
        lastAngle = desiredState.angle;

        double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_MODULE_SPEED;
        percentOutput = FS_Math.clamp(percentOutput, -1, 1);
        setDriveMotor(percentOutput);
    }

    private void setSteerMotor(double speed) {
        steerMotor.set(speed);
    }

    private void setDriveMotor(double speed) {
        driveMotor.set(speed);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                driveMotor.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_PERIMETER,
                getSteerAngle());
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }
}