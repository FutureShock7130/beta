package frc.FSLib2025.physics;

import edu.wpi.first.math.geometry.Rotation2d;

public class Physics {

    private final static double GRAVITY = 9.81;
    private final static double AIR_DENSITY = 1.225;

    public static double getProjectileYPosition(double xPosition, double initialVelocity, Rotation2d launchAngle) {
        double angleInRadians = launchAngle.getRadians();
        double time = xPosition / (initialVelocity * Math.cos(angleInRadians));
        double yPosition = (initialVelocity * Math.sin(angleInRadians) * time) - (0.5 * GRAVITY * time * time);
        return yPosition;
    }

    public static double calculateAirResistance(double dragCoefficient, double frontalArea, double velocity) {
        return 0.5 * dragCoefficient * AIR_DENSITY * frontalArea * velocity * velocity;
    }

}
