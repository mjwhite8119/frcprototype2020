package frc.robot.utilities;

import frc.robot.Constants;

public class ProjectileTrajectory {

    final double gravity = 9.80665;

    public ProjectileTrajectory(double distance, double angle, double velocity){
        
    }

    // Calculate velocity given distance and angle
    public double calculateVelocity(double distance, double angleDegrees) {
        // Convert to radians since that's what the Math functions use
        double theta = Math.abs(Math.toRadians(angleDegrees));

        // Distance y is the height of the target
        double y = Constants.RobotMap.kRelativeTargetHeight;
        double x = distance;

        // Now run the magic formular to get velocity
        double velocity = Math.sqrt(gravity*x*x/ 2*(y - Math.tan(theta)))/Math.cos(theta);
        
        return velocity;
    }

    // Calculate angle given distance and velocity
    public double calculateAngle(double distance, double velocity) {

        // Distance y is the height of the target
        double x = distance;
        double result = (x*gravity)/velocity*velocity;
        double theta = Math.asin(result)/2;
        double angleDegrees = Math.toDegrees(theta);

        return angleDegrees;
    }
}    