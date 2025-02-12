

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.FSLib2025.chassis.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class SwerveConstants {
        public static final double DEADBAND = 0.05;
        public static final double MAX_MODULE_SPEED = 5.0;
        public static final double MAX_MODULE_ROTATIONAL_SPEED = Units.degreesToRadians(360 * 1.15);
        public static final double WHEEL_BASE = 0.583; // 0.5715

        public static final int PIGEON_ID = 40;

        public static final double STEER_MOTOR_KP = 0.010;
        public static final double STEER_MOTOR_KI = 0.08; // 0.05
        public static final double STEER_MOTOR_KD = 0.015;
        public static final double STEER_MOTOR_WINDUP = 0.0;
        public static final int STEER_MOTOR_LIMIT = 0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.122449;
        public static final double DRIVE_WHEEL_DIAMETERS = 0.0964511800486235; // meters (4 * 0.0254)
        public static final double DRIVE_WHEEL_PERIMETER = Math.PI * DRIVE_WHEEL_DIAMETERS; // meters

        public static final Translation2d[] MODULE_TRANSLATOIN_METERS = new Translation2d[] {
                new Translation2d(WHEEL_BASE / 2.0, WHEEL_BASE / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -WHEEL_BASE / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, WHEEL_BASE / 2.0)
        };

        public static final SwerveModuleConstants LF_MOD_CONSTANTS = new SwerveModuleConstants();
        static {
            LF_MOD_CONSTANTS.DriveMotorId = 21;
            LF_MOD_CONSTANTS.SteerMotorId = 22;
            LF_MOD_CONSTANTS.CANcoderId = 0;
            LF_MOD_CONSTANTS.CANcoderOffset = 0.126465;
        }

        public static final SwerveModuleConstants RF_MOD_CONSTANTS = new SwerveModuleConstants();
        static {
            RF_MOD_CONSTANTS.DriveMotorId = 31;
            RF_MOD_CONSTANTS.SteerMotorId = 32;
            RF_MOD_CONSTANTS.CANcoderId = 1;
            RF_MOD_CONSTANTS.CANcoderOffset = 0.303955;
        }

        public static final SwerveModuleConstants RR_MOD_CONSTANTS = new SwerveModuleConstants();
        static {
            RR_MOD_CONSTANTS.DriveMotorId = 1;
            RR_MOD_CONSTANTS.SteerMotorId = 2;
            RR_MOD_CONSTANTS.CANcoderId = 2;
            RR_MOD_CONSTANTS.CANcoderOffset = -0.043457;
        }

        public static final SwerveModuleConstants LR_MOD_CONSTANTS = new SwerveModuleConstants();
        static {
            LR_MOD_CONSTANTS.DriveMotorId = 11;
            LR_MOD_CONSTANTS.SteerMotorId = 12;
            LR_MOD_CONSTANTS.CANcoderId = 3;
            LR_MOD_CONSTANTS.CANcoderOffset = 0.125977;
        }
    }

    public static class Vision {
        // public static final String OPCam = "OrangePiCamera";
        public static final String ATCam = "AprilTagCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRobotToOPCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));
        public static final Transform3d kRobotToATCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(28 / 2), -0.025, 0.71), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.5, 0.5, 0.8);
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(1, 1, 1);
    }

    public static final class FieldConstants {
        // starting pose
        public static final Pose2d Left = new Pose2d(new Translation2d(7.231, 7.615), Rotation2d.fromDegrees(180));
        public static final Pose2d Mid = new Pose2d(new Translation2d(8.068, 6.178), Rotation2d.fromDegrees(180));
        public static final Pose2d Right = new Pose2d(new Translation2d(8.068, 5.067), Rotation2d.fromDegrees(180));
        // REEF scoring pose
        public static final Pose2d A = new Pose2d(new Translation2d(3.180, 4.082), Rotation2d.fromDegrees(0));
        public static final Pose2d B = new Pose2d(new Translation2d(3.109, 3.847), Rotation2d.fromDegrees(0));
        public static final Pose2d C = new Pose2d(new Translation2d(3.654, 2.885), Rotation2d.fromDegrees(60));
        public static final Pose2d D = new Pose2d(new Translation2d(3.920, 2.731), Rotation2d.fromDegrees(60));
        public static final Pose2d E = new Pose2d(new Translation2d(5.058, 2.713), Rotation2d.fromDegrees(120));
        public static final Pose2d F = new Pose2d(new Translation2d(5.325, 2.880), Rotation2d.fromDegrees(120));
        public static final Pose2d G = new Pose2d(new Translation2d(5.883, 3.851), Rotation2d.fromDegrees(180));
        public static final Pose2d H = new Pose2d(new Translation2d(5.883, 4.190), Rotation2d.fromDegrees(180));
        public static final Pose2d I = new Pose2d(new Translation2d(5.341, 5.341), Rotation2d.fromDegrees(-120));
        public static final Pose2d J = new Pose2d(new Translation2d(5.072, 5.320), Rotation2d.fromDegrees(-120));
        public static final Pose2d K = new Pose2d(new Translation2d(3.887, 5.375), Rotation2d.fromDegrees(-60));
        public static final Pose2d L = new Pose2d(new Translation2d(3.623, 5.215), Rotation2d.fromDegrees(-60));
        // Algae & Coral
        public static final Pose2d LA = new Pose2d(new Translation2d(1.843, 5.848), Rotation2d.fromDegrees(180));
        public static final Pose2d LC = new Pose2d(new Translation2d(1.843, 5.848), Rotation2d.fromDegrees(180));
        public static final Pose2d MA = new Pose2d(new Translation2d(1.843, 4.020), Rotation2d.fromDegrees(180));
        public static final Pose2d MC = new Pose2d(new Translation2d(1.843, 4.020), Rotation2d.fromDegrees(180));
        public static final Pose2d RA = new Pose2d(new Translation2d(1.843, 2.166), Rotation2d.fromDegrees(180));
        public static final Pose2d RC = new Pose2d(new Translation2d(1.843, 2.166), Rotation2d.fromDegrees(180));
        // PROCESSOR
        public static final Pose2d PRO = new Pose2d(new Translation2d(6.347, 0.573), Rotation2d.fromDegrees(-90));
        // Coral Station
        public static final Pose2d CSR = new Pose2d(new Translation2d(1.666, 0.693), Rotation2d.fromDegrees(-126));
        public static final Pose2d CSL = new Pose2d(new Translation2d(1.666, 7.338), Rotation2d.fromDegrees(126));
    }
}