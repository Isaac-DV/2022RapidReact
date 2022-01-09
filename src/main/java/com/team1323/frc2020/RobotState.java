package com.team1323.frc2020;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.team1323.frc2020.subsystems.Swerve;
import com.team1323.frc2020.vision.GoalTracker;
import com.team1323.frc2020.vision.GoalTracker.TrackReport;
import com.team1323.frc2020.vision.GoalTracker.TrackReportComparator;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.frc2020.vision.TargetInfo;
import com.team1323.lib.util.CircularBuffer;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team1323.lib.util.Kinematics;
import com.team1323.lib.util.MovingAverageTwist2d;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
    private static RobotState instance = new RobotState();
    public static RobotState getInstance(){
        return instance;
    }
    
    private static final int kObservationBufferSize = 100;
    
    public static final Pose2d kVehicleToTurretFixed = new Pose2d(
    new Translation2d(Constants.Turret.kXOffset, Constants.Turret.kYOffset),
    Rotation2d.fromDegrees(0.0)
    );
    
    public static final Pose2d kTurretRotatingToCamera = new Pose2d(
    new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), 
    new Rotation2d()
    );
    
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_; 
    private CircularBuffer target_orientation;
    private Twist2d vehicle_velocity_;
    private MovingAverageTwist2d vehicle_velocity_averaged_;
    
    private GoalTracker goal_tracker_;
    private Rotation2d camera_pitch_correction_;
    private Rotation2d camera_yaw_correction_;
    private double differential_height_;
    private double distance_driven_;
    private double degrees_rotated_;
    private ShooterAimingParameters cached_shooter_aiming_params_ = null;

    private Translation2d lastKnownTargetPosition = new Translation2d();
    public Translation2d lastKnownTargetPosition(){ return lastKnownTargetPosition; }
    
    public double distanceToTarget(){
        return getLatestFieldToVehicle().getValue().transformBy(kVehicleToTurretFixed).getTranslation().distance(lastKnownTargetPosition);
    }

    public Rotation2d estimatedGyroDrift() {
        return Rotation2d.fromDegrees(degrees_rotated_ / 360.0 * Constants.kGyroDriftPerRotation);
    }
    
    private boolean seesTarget = false;
    public boolean seesTarget(){
        return seesTarget;
    }
    
    private RobotState() {
        reset(0, new Pose2d(), new Rotation2d());
    }
    
    /**
    * Resets the field to robot transform (robot's position on the field)
    */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle, Rotation2d initial_turret_rotation) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        turret_rotation_.put(new InterpolatingDouble(start_time), initial_turret_rotation);
        vehicle_velocity_ = Twist2d.identity();
        vehicle_velocity_averaged_ = new MovingAverageTwist2d(25);
        target_orientation = new CircularBuffer(20);
        goal_tracker_ = new GoalTracker();
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        differential_height_ = Constants.kVisionTargetHeight - Constants.kCameraZOffset;
        distance_driven_ = 0.0;
        degrees_rotated_ = 0.0;
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
    
    /**
     * @return robots current calculated position on the field with a timestamp and its pose
     */
    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        /*return getLatestFieldToVehicle().getValue().
            transformBy(Pose2d.exp(vehicle_velocity_averaged_.getAverage().scaled(lookahead_time)));*/
        Twist2d deltaPose = vehicle_velocity_averaged_.getAverage().scaled(lookahead_time);
        return getLatestFieldToVehicle().getValue().
            transformBy(new Pose2d(deltaPose.dx, deltaPose.dy, Rotation2d.fromRadians(deltaPose.dtheta)));
    }
    
    /**
     * @param timestamp current time on the FPGA
     * @return Turrets estimated rotation at a specific timestamp via the FPGA
     */
    public synchronized Rotation2d getTurretRotation(double timestamp) {
        return turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp));
    }
    
    /**
     * @return Turrets last estimated position with its timestamp
     */
    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
        return turret_rotation_.lastEntry();
    }
    
    /**
     * @param timestamp Any timestamp via the FPGA
     * @return Turret's pose relative to the field. Simply transforms the robots position on the field by 
     * the turret's position on the robot and it's estimated rotation based on the same timestamp passed in
     */
    public synchronized Pose2d getFieldToTurretRotated(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        return field_to_vehicle_.getInterpolated(key).transformBy(kVehicleToTurretFixed).transformBy(Pose2d.fromRotation(turret_rotation_.getInterpolated(key)));
    }
    
    /**
     * @param timestamp Any timestamp via the FPGA
     * @return Camera's pose relative to the field. Translates the estimated pose of the turret on the field by the
     * camera's position relative to the center of the turret
     */
    public synchronized Pose2d getFieldToCamera(double timestamp) {
        return getFieldToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }

    private Translation2d getFieldToGoal(Pose2d field_to_camera, TargetInfo target) {
        double ydeadband = target.getY();
                
        // Compensate for camera yaw
        double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
        double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
        double zyaw = target.getZ();
        
        // Compensate for camera pitch
        double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
        double yr = yyaw;
        double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();
        
        // find intersection with the goal
        double scaling = differential_height_ / zr;
        double distance = Math.hypot(xr, yr) * scaling;
        Rotation2d angle = new Rotation2d(xr, yr, true);

        return field_to_camera.transformBy(Pose2d
        .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
        .getTranslation();
    }
    
    /**
     * Adds vision update
     * @param timestamp any timestamp via the FPGA
     * @param vision_update List of vision data in x y and z of the seen target or not seen for that matter.
     * Possible that the vision_update is empty
     */
    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        Pose2d field_to_camera = getFieldToCamera(timestamp);
        if (!(vision_update == null || vision_update.isEmpty())) {
            seesTarget = true;
            field_to_goals.add(getFieldToGoal(field_to_camera, vision_update.get(0)));
            /*Translation2d leftCorner = getFieldToGoal(field_to_camera, vision_update.get(1));
            Translation2d rightCorner = getFieldToGoal(field_to_camera, vision_update.get(2));
            Translation2d delta = rightCorner.translateBy(leftCorner.inverse());
            target_orientation.addValue(delta.direction().rotateBy(Rotation2d.fromDegrees(-90.0)).getDegrees());*/
        }else{
            seesTarget = false;
        }
        synchronized (this) {
            goal_tracker_.update(timestamp, field_to_goals);
        }
    }
        
        /**
         * Gets the last aiming parameters seen by the robot. Since it is an optional, if the robot hasn't seen the target then it is possible that it's null
         * @return Range, timestamp in FPGA time of when the target was last seen, stability of the target, last estimated robot to goal rotation
         * and the last rotation of the target's orientation
         */
        public synchronized Optional<ShooterAimingParameters> getCachedAimingParameters() {
            return cached_shooter_aiming_params_ == null ? Optional.empty() : Optional.of(cached_shooter_aiming_params_);
        }

        private boolean isAimingAtInnerPort = false;
        private boolean isTrackerEmpty = true;
        public synchronized Optional<ShooterAimingParameters> getAimingParameters(boolean aimInnerPort) {
			List<TrackReport> reports = goal_tracker_.getTracks();
			if (!reports.isEmpty()) {

                TrackReportComparator comparator = new TrackReportComparator(
                    Constants.kTrackStabilityWeight,
                    Constants.kTrackAgeWeight,
                    Constants.kTrackSwitchingWeight,
                    -1, Timer.getFPGATimestamp());
                reports.sort(comparator);

                TrackReport report = reports.get(0);
                Pose2d vehicle_pose = getPredictedFieldToVehicle(Constants.kPosePredictionTime);
                //Rotation2d orientation = Rotation2d.fromDegrees(target_orientation.getAverage());

                Translation2d outerPortPosition = lastKnownTargetPosition = report.field_to_goal;
                Translation2d innerPortPosition = new Pose2d(report.field_to_goal, Constants.kPortTargetOrientation/*orientation*/)
                    .transformBy(Pose2d.fromTranslation(Constants.kOuterPortToInnerPort)).getTranslation();

                Translation2d turret_to_outer_port = vehicle_pose.transformBy(kVehicleToTurretFixed)
                    .getTranslation().inverse().translateBy(outerPortPosition);
                Translation2d turret_to_inner_port = vehicle_pose.transformBy(kVehicleToTurretFixed)
                    .getTranslation().inverse().translateBy(innerPortPosition);

                Pose2d turret_to_goal_robot_centric = vehicle_pose.transformBy(kVehicleToTurretFixed)
                    .inverse().transformBy(Pose2d.fromTranslation(outerPortPosition));
                
                if (aimInnerPort) {
                    turret_to_goal_robot_centric = vehicle_pose.transformBy(kVehicleToTurretFixed)
                        .inverse().transformBy(Pose2d.fromTranslation(innerPortPosition));

                    if (Math.abs(turret_to_inner_port.direction().distance(Constants.kPortTargetOrientation /*orientation*/)) > Math.toRadians(20.0)) {
                        turret_to_goal_robot_centric = vehicle_pose.transformBy(kVehicleToTurretFixed).inverse().transformBy(Pose2d.fromTranslation(outerPortPosition));
                        //System.out.println("Aiming at the OUTER port");
                        isAimingAtInnerPort = false;
                    } else {
                        isAimingAtInnerPort = true;
                        //System.out.println("Aiming at the INNER port");
                    }
                }
                isTrackerEmpty = false;

                //System.out.println("TURRET TO INNER PORT ANGLE: " + turret_to_inner_port.direction().getDegrees());

				ShooterAimingParameters params = new ShooterAimingParameters(turret_to_outer_port.norm(), 
						turret_to_goal_robot_centric.getTranslation().direction(), turret_to_outer_port, report.latest_timestamp, report.stability);
				cached_shooter_aiming_params_ = params;

				return Optional.of(params);
			} else {
                isTrackerEmpty = true;
                //System.out.println("Empty goal tracker");
				return Optional.empty();
			}
		}
        
        /**
         * Gets the robots relation to the vision target
         * @param aimingParameters takes in the aiming parameters which consist of: range, FPGA timestamp of last time target was seen, stability, robot to goal rotation, rotation of target
         * @param orientation ending rotation of the robot
         * @param endTranslation ending translation of the target. Can be used to translate the robot forward/backward and left/right of the center of target
         * @return Ending pose that the robot should be in if the aimingParameters are valid. returned value can be empty.
         */
        public synchronized Optional<Pose2d> getRobotScoringPosition(Optional<ShooterAimingParameters> aimingParameters, Rotation2d orientation, Translation2d endTranslation){
            List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
            if(!targetPositions.isEmpty() && aimingParameters.isPresent()){
                Translation2d targetPosition = targetPositions.get(0).getTranslation();
                //SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), aimingParameters.get().getTargetOrientation().getDegrees(), 0.0}); 
                Pose2d orientedTargetPosition = new Pose2d(targetPosition, orientation)/*.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)))*/;
                Pose2d robotScoringPosition = orientedTargetPosition.transformBy(Pose2d.fromTranslation(endTranslation));
                
                return Optional.of(robotScoringPosition);
            }
            return Optional.empty();
        }
        
        /**
         * Resets the robots position on the field using vision. Uses the targetPosition to estimate where the robot
         * is relative to where the robot sees the vision target.
         * @param targetPosition distance in x and y coordinates that the robot is away from the vision target
         */
        public synchronized void resetRobotPosition(Translation2d targetPosition){
            List<TrackReport> reports = goal_tracker_.getTracks();
            if (!reports.isEmpty()) {
                TrackReport report = reports.get(0);
                Translation2d robotFrameToFieldFrame = report.field_to_goal.inverse().translateBy(targetPosition);
                if(robotFrameToFieldFrame.norm() <= 5.0){
                    Swerve.getInstance().resetPosition(new Pose2d(Swerve.getInstance().getPose().getTranslation().translateBy(robotFrameToFieldFrame), Swerve.getInstance().getPose().getRotation()));
                    System.out.println("Coordinates corrected by " + robotFrameToFieldFrame.norm() + " inches");
                }else{
                    System.out.println("Coordinate correction too large: " + robotFrameToFieldFrame.norm());
                }
            }else{
                System.out.println("Vision did not detect target");
            }
        }
        
        /**
         * @return Lag between the last seen goal and the currently seen one
         */
        public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
            List<Pose2d> rv = new ArrayList<>();
            for (TrackReport report : goal_tracker_.getTracks()) {
                rv.add(Pose2d.fromTranslation(report.field_to_goal));
            }
            return rv;
        }
        
        public synchronized void clearVisionTargets(){
            goal_tracker_.clearTracks();
        }
        
        public synchronized void resetVision() {
            goal_tracker_.reset();
        }
        
        /**
         * Adds a new pose estimation for the current FPGA timestamp
         * @param timestamp
         * @param observation
         */
        public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
            field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
        }
        
        /**
         * Adds a new turret rotation estimation for the current FPGA timestamp
         * @param timestamp
         * @param observation
         */
        public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
            turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
        }

        public synchronized void addObservations(double timestamp, Pose2d pose, Twist2d velocity, Rotation2d turretAngle) {
            degrees_rotated_ += Math.abs(Math.toDegrees(getLatestFieldToVehicle().getValue().getRotation().distance(pose.getRotation())));
            addFieldToVehicleObservation(timestamp, pose);
            addTurretRotationObservation(timestamp, turretAngle);
            vehicle_velocity_ = velocity;
            if (Math.abs(vehicle_velocity_.dtheta) < 2.0 * Math.PI) {
                vehicle_velocity_averaged_.add(vehicle_velocity_);
            } else {
                vehicle_velocity_averaged_.add(new Twist2d(vehicle_velocity_.dx, vehicle_velocity_.dy, 0.0));
            }
        }

        public synchronized void addObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity, Rotation2d gyroAngle, Rotation2d turretAngle) {
            addTurretRotationObservation(timestamp, turretAngle);
            Pose2d newPose = Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), displacement);
            addFieldToVehicleObservation(timestamp, new Pose2d(newPose.getTranslation(), gyroAngle));
        }
        
        public synchronized double getDistanceDriven() {
            return distance_driven_;
        }
        
        public void outputToSmartDashboard(){
            SmartDashboard.putBoolean("Sees Target", seesTarget);

            /*Twist2d velocity = vehicle_velocity_averaged_.getAverage();
            SmartDashboard.putNumber("Swerve Velocity dx", velocity.dx);
            SmartDashboard.putNumber("Swerve Velocity dy", velocity.dy);
            SmartDashboard.putNumber("Swerve Velocity dtheta", Math.toDegrees(velocity.dtheta));

            SmartDashboard.putNumber("Swerve Degrees Rotated", degrees_rotated_);*/
            
            if(Settings.debugVision()){
                List<Pose2d> poses = getCaptureTimeFieldToGoal();
                for (Pose2d pose : poses) {
                    // Only output first goal
                    SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().x());
                    SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().y());
                    
                    break;
                }
                Optional<ShooterAimingParameters> aiming_params = /*getCachedAimingParameters();*/getAimingParameters(false);
                if (aiming_params.isPresent()) {
                    SmartDashboard.putNumber("goal_range", aiming_params.get().getRange());
                    SmartDashboard.putNumber("goal_theta", aiming_params.get().getTurretAngle().getDegrees());
                    SmartDashboard.putBoolean("Aiming Inner Port", isAimingAtInnerPort);
                    SmartDashboard.putBoolean("Tracker Is Empty", isTrackerEmpty);
                } else {
                    SmartDashboard.putNumber("goal_range", 0.0);
                    SmartDashboard.putNumber("goal_theta", 0.0);
                }
                /*if (target_orientation.getNumValues() > 0) {
                    SmartDashboard.putNumber("goal_orientation", target_orientation.getAverage());
                } else {
                    SmartDashboard.putNumber("goal_orientation", target_orientation.getAverage());
                }*/
            }
        }
    }
    