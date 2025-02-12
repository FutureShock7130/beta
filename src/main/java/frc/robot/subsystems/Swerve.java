package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.Vision;

public class Swerve extends SubsystemBase {

    private static Swerve instance = null;
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON_ID, "GTX7130");

    private SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.LF_MOD_CONSTANTS),
            new SwerveModule(1, SwerveConstants.RF_MOD_CONSTANTS),
            new SwerveModule(2, SwerveConstants.RR_MOD_CONSTANTS),
            new SwerveModule(3, SwerveConstants.LR_MOD_CONSTANTS)
    };
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATOIN_METERS);
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroYaw(), getModulePositions());
    private SwerveDrivePoseEstimator poseEstimator;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private Field2d field = new Field2d();

    private TalonFX driveMotor = modules[0].driveMotor;
    private final VoltageOut m_Volreq = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())
        ), new Mechanism((volts) -> driveMotor.setControl(m_Volreq.withOutput(volts.in(Volts))), null, this)
    );

    public Swerve() {
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            Vision.kStateStdDevs,
            Vision.kVisionMeasurementStdDevs);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (config == null) {
            throw new RuntimeException("Failed to load config");
        }

        AutoBuilder.configure(
                this::getOdometryPosition,
                this::setOdometryPosition,
                this::getRobotRelativSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5,0, 0.5, 0.0), // 18
                        new PIDConstants(40, 0.0, 5)), // 5
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
        );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getRobotRelativSpeeds(); // Method to get current robot-relative chassis speeds
        SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

        // odometry.resetPose(new Pose2d());
        // pigeon.reset();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.01);
    //     SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    //     setModuleStates(targetStates);
    // }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );
        setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public void setGyroYaw(Rotation2d yaw) {
        pigeon.setYaw(yaw.getDegrees());
    }

    public Pose2d getOdometryPosition() {
        return odometry.getPoseMeters();
    }

    public void setOdometryPosition(Pose2d pose) {
        odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getEstimatedPosition() {
    return poseEstimator.getEstimatedPosition();
    }

    public void resetPosewithVision(Pose2d visionPose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), visionPose);;
    }

    public ChassisSpeeds getRobotRelativSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getGyroYaw());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public double[] getModuleAngle() {
        double[] angles = new double[4];
        for (int i = 0; i < 4; i++) {
            angles[i] = modules[i].getSteerAngle().getDegrees();
        }
        return angles; // Return the array
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("desiredStates must have length 4");
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_MODULE_SPEED);
        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.ModuleNumber]);
        }
    }

    public void stop() {
        driveRobotRelative(new ChassisSpeeds());
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public Command safetystop() {
        return Commands.run(() -> {driveRobotRelative(new ChassisSpeeds());}, this);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), getModulePositions());
        odometry.update(getGyroYaw(), getModulePositions());
        
        SmartDashboard.putNumber("gyro (deg)", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("swerve odometry x", getOdometryPosition().getX());
        SmartDashboard.putNumber("swerve odometry y", getOdometryPosition().getY());
        SmartDashboard.putNumber("Module 1 Angle", getModuleAngle()[0]);
        SmartDashboard.putNumber("Module 2 Angle", getModuleAngle()[1]);
        SmartDashboard.putNumber("Module 3 Angle", getModuleAngle()[2]);
        SmartDashboard.putNumber("Module 4 Angle", getModuleAngle()[3]);
    }
}
