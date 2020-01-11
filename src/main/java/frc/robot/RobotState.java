
package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.GoalTracker.TrackReport;
import frc.robot.lib.vision.AimingParameters;
import frc.robot.lib.vision.VisionList;
import frc.robot.lib.vision.VisionTarget;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Translation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Map;
import java.util.List;

public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    private static final int kObservationBufferSize = 100;

    private static final Pose2d kVehicleToCamera = new Pose2d(
            new Translation2d(Constants.X_DIR_OFFSET_IN_METERS, Constants.Y_DIR_OFFSET_IN_METERS), Rotation2d.fromDegrees(Constants.INITIAL_Z_AXIS_ROTATION));

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private GoalTracker goal_tracker_;
    private double distance_driven_;

    private RobotState() {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */

     
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        //Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        goal_tracker_ = new GoalTracker();
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */

     
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized Pose2d getFieldToCamera(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCamera);
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public void addVisionUpdate(double timestamp, VisionList vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        Pose2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.getTargets().isEmpty())) {
            for (VisionTarget target : vision_update.getTargets()) {
                    field_to_goals.add(field_to_camera
                            .transformBy(Pose2d
                                    .fromTranslation(new Translation2d(target.getDistance() * Math.cos(target.getAngleInRadians()), target.getDistance() * Math.sin(target.getAngleInRadians()))))
                            .getTranslation());
                
            }
        }
        
        synchronized (this) {
            goal_tracker_.update(timestamp, field_to_goals);
        }
    }

    public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
        List<Pose2d> rv = new ArrayList<>();
        for (TrackReport report : goal_tracker_.getTracks()) {
            rv.add(Pose2d.fromTranslation(report.field_to_goal));
        }
        return rv;
    }

    public synchronized AimingParameters getAimingParameters() {
        List<TrackReport> reports = goal_tracker_.getTracks();
        if (!reports.isEmpty()) {
            TrackReport report = reports.get(0);
            Translation2d robot_to_goal = getLatestFieldToVehicle().getValue().getTranslation().inverse()
                    .translateBy(report.field_to_goal);
            Rotation2d robot_to_goal_rotation = Rotation2d
                    .fromRadians(Math.atan2(robot_to_goal.y(), robot_to_goal.x()));

            AimingParameters params = new AimingParameters(robot_to_goal.norm(), robot_to_goal_rotation,
                    report.latest_timestamp, report.stability);

            return params;
        }
        else
            return null;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, double strafe_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematicsMecanum(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, strafe_encoder_delta_distance,
                current_gyro_angle);
        distance_driven_ += Math.sqrt(Math.pow(delta.dx, 2) + Math.pow(delta.dy, 2));
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
    }
}
