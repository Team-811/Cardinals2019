
package frc.robot.subsystems;

import frc.robot.Kinematics;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.commands.RobotStateEstimator.*;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  This class uses the RobotState class to keep record of the robots position
 over time.  This was written by team 254
 */

public class RobotStateEstimator extends Subsystem {

    private static RobotStateEstimator instance = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
      return instance;
    }


    private RobotState robot_state_ = RobotState.getInstance();
    private Drivetrain drive = Drivetrain.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private double strafe_encoder_prev_distance_ = 0.0;

    public RobotStateEstimator() 
    {
        left_encoder_prev_distance_ = drive.getLeftEncoder();
        right_encoder_prev_distance_ = drive.getRightEncoder();
        strafe_encoder_prev_distance_ = drive.getStrafeEncoder();
    }

    
    public void runRobotStateEstimator(double timestamp)  {
        final double left_distance = drive.getLeftEncoder();
        final double right_distance = drive.getRightEncoder();
        final double strafe_distance = drive.getStrafeEncoder();
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final double delta_strafe = strafe_distance - strafe_encoder_prev_distance_;
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(drive.getGyroAngle());
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                delta_left, delta_right, delta_strafe, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematicsMecanum(drive.getTopLeftVelocity(),
                drive.getBottomRightVelocity(), drive.getStrafeVelocity());
        robot_state_.addObservations(timestamp, odometry_velocity,
                predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

        

        @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new EstimatePosition());
    }

}
