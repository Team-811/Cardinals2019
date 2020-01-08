package frc.robot;

import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;

import frc.robot.Constants;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).  This code was created by team 254 and was edited to fit our robot.  This is used in
 * the robotState class to help calculate robot position
 */

public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2d forwardKinematicsDifferential(double left_wheel_delta, double right_wheel_delta) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (Constants.wheelbase * Constants.trackScrubFactor);
        return forwardKinematicsDifferential(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    //Take the diagonal wheels so strafing does not impact rotation calculation
    public static Twist2d forwardKinematicsMecanum(double left_top_wheel_delta, double right_bottom_wheel_delta, double strafe_wheel_delta) {
        double delta_rotation = (right_bottom_wheel_delta - left_top_wheel_delta) / (Constants.wheelbase * Constants.trackScrubFactor);
        return forwardKinematicsMecanum(left_top_wheel_delta, right_bottom_wheel_delta, strafe_wheel_delta, delta_rotation);
    }

    /**
     * Forward kinematics using encoders and gyro
     */
    public static Twist2d forwardKinematicsDifferential(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2d(dx, 0.0, delta_rotation_rads);
    }

    public static Twist2d forwardKinematicsDifferential(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
                         Rotation2d current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    public static Twist2d forwardKinematicsMecanum(double left_wheel_delta, double right_wheel_delta, 
                        double strafe_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = strafe_wheel_delta;
        return new Twist2d(dx, dy, delta_rotation_rads);
    }

    public static Twist2d forwardKinematicsMecanum(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
                        double strafe_wheel_delta, Rotation2d current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = strafe_wheel_delta;
        return new Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose,
                                                    Twist2d forward_kinematics) {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics));
    }
}
