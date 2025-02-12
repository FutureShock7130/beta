package frc.FSLib2025.odometry;

public class TankDriveOdometry {

    private double x;      // Robot's X position
    private double y;      // Robot's Y position
    private double theta;  // Robot's heading angle (in radians)

    public TankDriveOdometry(double initialX, double initialY, double initialTheta) {
        this.x = initialX;
        this.y = initialY;
        this.theta = initialTheta;
    }

    public void updateOdometry(double leftDistance, double rightDistance, double wheelBase) {
        double deltaTheta = (rightDistance - leftDistance) / wheelBase;
        double deltaDistance = (leftDistance + rightDistance) / 2;

        // Update the robot's position
        double thetaMid = theta + (deltaTheta / 2.0); // Midpoint orientation for more accurate position update
        x += deltaDistance * Math.cos(thetaMid);
        y += deltaDistance * Math.sin(thetaMid);

        theta += deltaTheta;
        theta = normalizeAngle(theta);
    }

    // Normalize Angle to -pi to pi
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void resetOdometry(double newX, double newY, double newTheta) {
        this.x = newX;
        this.y = newY;
        this.theta = newTheta;
    }
}
