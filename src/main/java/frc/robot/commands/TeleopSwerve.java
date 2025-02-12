package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {

    private final Swerve swerve;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    private CommandXboxController controller;

    public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        if (controller.getHID().getAButtonPressed()) {
            swerve.setOdometryPosition(FieldConstants.A);
        }

        if (controller.getHID().getBackButtonPressed()) {
            swerve.setOdometryPosition(new Pose2d());
            swerve.setGyroYaw(new Rotation2d());
        }

        double slow = 1;
        if (controller.getHID().getLeftBumperButton() || controller.getHID().getRightBumperButton()) {
            slow = 0.5;
        }

        double deadband = 0.04;
        double xyMultiplier = 1;
        double zMultiplier = 0.75;

        xSpeed = xLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftY(), deadband));
        ySpeed = yLimiter.calculate(MathUtil.applyDeadband(-controller.getLeftX(), deadband));
        rotSpeed = rotLimiter.calculate(MathUtil.applyDeadband(-controller.getRightX(), deadband));
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
        // rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);
        swerve.drive(
                new Translation2d(xSpeed, ySpeed).times(SwerveConstants.MAX_MODULE_SPEED).times(xyMultiplier)
                        .times(slow),
                rotSpeed * SwerveConstants.MAX_MODULE_ROTATIONAL_SPEED * zMultiplier * slow,
                // 0,
                true);

        if (controller.getHID().getYButtonPressed()) {
            AutoBuilder.pathfindToPose(FieldConstants.A,
                    new PathConstraints(
                            5,
                            6,
                            5 * Math.PI,
                            4 * Math.PI))
                    .until(isDriverControl).schedule();
        }
        if (controller.getHID().getLeftTriggerAxis() > 0.4) {
            AutoBuilder.pathfindToPose(FieldConstants.CSL,
                    new PathConstraints(
                            5,
                            6,
                            5 * Math.PI,
                            4 * Math.PI))
                    .until(isDriverControl).schedule();
        }
        if (controller.getHID().getRightTriggerAxis() > 0.4) {
            AutoBuilder.pathfindToPose(FieldConstants.CSR,
                    new PathConstraints(
                            5,
                            6,
                            5 * Math.PI,
                            4 * Math.PI))
                    .until(isDriverControl).schedule();
        }

    }

    public BooleanSupplier isDriverControl = () -> {
        return Math.abs(controller.getLeftX()) >= 0.1 || Math.abs(controller.getLeftY()) >= 0.1;
    };

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false);
    }
}
