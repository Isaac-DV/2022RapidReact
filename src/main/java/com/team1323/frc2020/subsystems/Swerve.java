package com.team1323.frc2020.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.DriveMotionPlanner;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.math.vectors.VectorField;
import com.team1323.lib.util.DriveSignal;
import com.team1323.lib.util.Kinematics;
import com.team1323.lib.util.SwerveHeadingController;
import com.team1323.lib.util.SwerveInverseKinematics;
import com.team1323.lib.util.SynchronousPIDF;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.wpilib.SwerveDriveKinematics;
import com.wpilib.SwerveDriveOdometry;
import com.wpilib.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem{
	//Instance declaration
	private static Swerve instance = null;
	public static Swerve getInstance(){
		if(instance == null)
		instance = new Swerve();
		return instance;
	}
	
	//Module declaration
	public SwerveDriveModule frontRight, frontLeft, rearLeft, rearRight;
	List<SwerveDriveModule> modules;
	List<SwerveDriveModule> positionModules;
	
	//Evade maneuver variables
	Translation2d clockwiseCenter = new Translation2d();
	Translation2d counterClockwiseCenter = new Translation2d();
	boolean evading = false;
	boolean evadingToggled = false;
	public void toggleEvade(){
		evading = !evading;
		evadingToggled = true;
	}
	
	//Heading controller methods
	Pigeon pigeon;
	SwerveHeadingController headingController = new SwerveHeadingController();
	public void temporarilyDisableHeadingController(){
		headingController.temporarilyDisable();
	}
	public double getTargetHeading(){
		return headingController.getTargetHeading();
	}
	
	//Vision dependencies
	RobotState robotState;
	Rotation2d visionTargetHeading = new Rotation2d();
	boolean visionUpdatesAllowed = true;
	Translation2d lastVisionEndTranslation = new Translation2d(0.0, 0.0);
	boolean useFixedVisionOrientation = false;
	Rotation2d fixedVisionOrientation = Rotation2d.fromDegrees(0.0);
	Rotation2d visionApproachAngle = Rotation2d.fromDegrees(0.0);
	double visionCutoffDistance = Constants.kClosestVisionDistance;
	public boolean isTracking(){
		return currentState == ControlState.VISION_PID;
	}
	Pose2d visionPIDTarget;
	SynchronousPIDF lateralPID = new SynchronousPIDF(0.04, 0.0, 0.0); // 0.05, 0.0, 0.0
	SynchronousPIDF forwardPID = new SynchronousPIDF(0.04, 0.0, 0.0); // 0.02, 0.0, 0.0
	boolean visionTargetAcquired = false;
	boolean visionTargetReached = false;
	
	boolean needsToNotifyDrivers = false;
	public boolean needsToNotifyDrivers(){
		if(needsToNotifyDrivers){
			needsToNotifyDrivers = false;
			return true;
		}		
		return false;
	}
	
	//Name says it all
	TrajectoryGenerator generator;
	
	//Odometry variables
	Pose2d pose;
	Twist2d velocity = Twist2d.identity();
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;
	public Pose2d getPose(){
		return pose;
	}
	public Twist2d getVelocity() {
		return velocity;
	}
	
	//Wpilib odometry
	SwerveDriveOdometry odometry;
	Pose2d wpiPose = new Pose2d();
	boolean outputWpiPose = false;
	
	// Module configuration variables (for beginnning of auto)
	boolean modulesReady = false;
	boolean alwaysConfigureModules = false;
	boolean moduleConfigRequested = false;
	public void requireModuleConfiguration(){
		modulesReady = false;
	}
	public void alwaysConfigureModules(){
		alwaysConfigureModules = true;
	}
	Pose2d startingPose = Constants.kRobotStartingPose;
	public void setStartingPose(Pose2d newPose){
		startingPose = newPose;
	}
	
	//Trajectory variables
	DriveMotionPlanner motionPlanner;
	public double getRemainingProgress(){
		if(motionPlanner != null && getState() == ControlState.TRAJECTORY){
			return motionPlanner.getIterator().getRemainingProgress() / (motionPlanner.getIterator().getProgress() + motionPlanner.getIterator().getRemainingProgress());
		}
		return 0.0;
	}
	double rotationScalar;
	public void setRotationScalar(double scalar) {
		rotationScalar = scalar;
	}
	double trajectoryStartTime = 0;
	Translation2d lastTrajectoryVector = new Translation2d();
	public Translation2d getLastTrajectoryVector(){ return lastTrajectoryVector; }
	boolean hasStartedFollowing = false;
	boolean hasFinishedPath = false;
	public boolean hasFinishedPath(){
		return hasFinishedPath;
	}
	
	//Experimental
	VectorField vf;
	
	private Swerve(){
		/**
		 * 			   	  Front
		 *     __________________________
		 * 	   | M1 |               | M0 |
		 * 	   |____|               |____|
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |                         |
		 * 	   |____                _____|
		 * 	   | M2 |               | M3 |
		 *     |____|_______________|____|
		 * 		
		 */
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE,
		0, Constants.kFrontRightEncoderStartingPos, Constants.kVehicleToModuleZero, true);
		frontLeft = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE,
		1, Constants.kFrontLeftEncoderStartingPos, Constants.kVehicleToModuleOne, true);
		rearLeft = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE,
		2, Constants.kRearLeftEncoderStartingPos, Constants.kVehicleToModuleTwo, true);
		rearRight = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE,
		3, Constants.kRearRightEncoderStartingPos, Constants.kVehicleToModuleThree, true);
		
		modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
		positionModules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);

		/**
		 * frontLeft.invertDriveMotor(true)
		 * rearLeft.invertDriveMotor(true)
		 * 
		 * frontRight.invertDriveMotor(false)
		 * rearRight.invertDriveMotor(false)
		 */

		/*
		 frontLeft.invertDriveMotor(true);
		rearLeft.invertDriveMotor(true);
		frontRight.invertDriveMotor(false);
		rearRight.invertDriveMotor(false);
		*/

		frontLeft.invertDriveMotor(TalonFXInvertType.Clockwise);
		rearLeft.invertDriveMotor(TalonFXInvertType.Clockwise);
		frontRight.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearRight.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		
		pigeon = Pigeon.getInstance();
		
		pose = new Pose2d();
		distanceTraveled = 0;
		
		motionPlanner = new DriveMotionPlanner();
		
		robotState = RobotState.getInstance();
		
		odometry = new SwerveDriveOdometry(new SwerveDriveKinematics(Constants.kVehicleToModuleZero, 
		Constants.kVehicleToModuleOne, Constants.kVehicleToModuleTwo, Constants.kVehicleToModuleThree), 
		Rotation2d.identity());
		
		generator = TrajectoryGenerator.getInstance();
		
		lateralPID.setSetpoint(0.0);
		forwardPID.setSetpoint(0.0);
	}
	
	//Assigns appropriate directions for scrub factors
	public void setCarpetDirection(boolean standardDirection){
		modules.forEach((m) -> m.setCarpetDirection(standardDirection));
	}
	
	//Teleop driving variables
	private Translation2d translationalVector = new Translation2d();
	private double rotationalInput = 0;
	private Translation2d lastDriveVector = new Translation2d();
	private final Translation2d rotationalVector = Translation2d.identity();
	private double lowPowerScalar = 0.6;
	public void setLowPowerScalar(double scalar){
		lowPowerScalar = scalar;
	}
	private double maxSpeedFactor = 1.0;
	public void setMaxSpeed(double max){
		maxSpeedFactor = max;
	}
	private boolean robotCentric = false;
	
	//Swerve kinematics (exists in a separate class)
	private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
	public void setCenterOfRotation(Translation2d center){
		inverseKinematics.setCenterOfRotation(center);
	}
	
	//The swerve's various control states
	public enum ControlState{
		NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED,
		TRAJECTORY, VELOCITY, VISION_PID
	}
	private ControlState currentState = ControlState.NEUTRAL;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	
	/**
	* Main function used to send manual input during teleop.
	* @param x forward/backward input
	* @param y left/right input
	* @param rotate rotational input
	* @param robotCentric gyro use
	* @param lowPower scaled down output
	*/
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
		Translation2d translationalInput = new Translation2d(x, y);
		double inputMagnitude = translationalInput.norm();
		
		/* Snap the translational input to its nearest pole, if it is within a certain threshold 
		of it. */
		double threshold = Math.toRadians(10.0);
		if(Math.abs(translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold){
			translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
		}
		
		/* Scale x and y by applying a power to the magnitude of the vector they create, in order
		to make the controls less sensitive at the lower end. */
		double deadband = 0.05;
		inputMagnitude = Util.scaledDeadband(inputMagnitude, 1.0, deadband);
		final double power = (lowPower) ? 1.75 : 1.5;
		inputMagnitude = Math.pow(inputMagnitude, power);
		inputMagnitude = Util.deadBand(inputMagnitude, 0.05);
		translationalInput = Translation2d.fromPolar(translationalInput.direction(), inputMagnitude);
		
		rotate = Util.scaledDeadband(rotate, 1.0, deadband);
		rotate = Math.pow(Math.abs(rotate), 1.75)*Math.signum(rotate);
		
		translationalInput = translationalInput.scale(maxSpeedFactor);
		rotate *= maxSpeedFactor;
		
		translationalVector = translationalInput;
		
		if(lowPower){
			translationalVector = translationalVector.scale(lowPowerScalar);
			rotate *= lowPowerScalar;
		}else{
			rotate *= 0.8;
		}
		
		if(!Util.epsilonEquals(rotate, 0.0) && Util.epsilonEquals(rotationalInput, 0.0)){
			headingController.disable();
		}else if(Util.epsilonEquals(rotate, 0.0) && !Util.epsilonEquals(rotationalInput, 0.0)){
			headingController.temporarilyDisable();
		}
		
		rotationalInput = rotate;
		
		if(!Util.epsilonEquals(translationalInput.norm(), 0.0)){
			if (headingController.getState() == SwerveHeadingController.State.Snap || 
				headingController.getState() == SwerveHeadingController.State.Stationary)
				headingController.setStabilizationTarget(headingController.getTargetHeading());

			if(isTracking() || currentState == ControlState.POSITION){
				if(Math.abs(translationalInput.direction().distance(visionTargetHeading)) > Math.toRadians(120.0)){
					//setState(ControlState.MANUAL);
				}
			} else if(currentState != ControlState.MANUAL){
				setState(ControlState.MANUAL);
			}
		}else if(!Util.epsilonEquals(rotationalInput, 0.0)){
			if(currentState != ControlState.MANUAL && currentState != ControlState.TRAJECTORY && currentState != ControlState.VISION_PID && currentState != ControlState.POSITION){
				setState(ControlState.MANUAL);
			}
		}
		
		if(inputMagnitude > 0.1)
		lastDriveVector = new Translation2d(x, y);
		else if(translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0){
			lastDriveVector = rotationalVector;
		}
		
		this.robotCentric = robotCentric;

		//System.out.println("Swerve translational input: " + translationalVector.toString());
	}
	
	//Possible new control method for rotation
	public Rotation2d averagedDirection = Rotation2d.identity();
	public void resetAveragedDirection(){ averagedDirection = pose.getRotation(); }
	public void setAveragedDirection(double degrees){ averagedDirection = Rotation2d.fromDegrees(degrees); }
	public final double rotationDirectionThreshold = Math.toRadians(5.0);
	public final double rotationDivision = 1.0;
	
	public synchronized void updateControllerDirection(Translation2d input){
		if(Util.epsilonEquals(input.norm(), 1.0, 0.1)){
			Rotation2d direction = input.direction();
			double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
			averagedDirection = Rotation2d.fromDegrees(roundedDirection);
		}
	}
	
	//Various methods to control the heading controller
	public synchronized void rotate(double goalHeading){
		if(translationalVector.x() == 0 && translationalVector.y() == 0)
		rotateInPlace(goalHeading);
		else
		headingController.setStabilizationTarget(
		Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlace(double goalHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(
		Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void rotateInPlaceAbsolutely(double absoluteHeading){
		setState(ControlState.ROTATION);
		headingController.setStationaryTarget(absoluteHeading);
	}
	
	public void setPathHeading(double goalHeading){
		headingController.setSnapTarget(
		Util.placeInAppropriate0To360Scope(
		pose.getRotation().getUnboundedDegrees(), goalHeading));
	}
	
	public void setAbsolutePathHeading(double absoluteHeading){
		headingController.setSnapTarget(absoluteHeading);
	}

	public boolean isGoingToPole() {
		Rotation2d targetHeading = Rotation2d.fromDegrees(headingController.getTargetHeading());
		return targetHeading.equals(Rotation2d.fromDegrees(0.0)) || targetHeading.equals(Rotation2d.fromDegrees(180.0));
	}

	public double closestPole() {
		if (Math.abs(pose.getRotation().distance(Rotation2d.fromDegrees(180.0))) < 
			Math.abs(pose.getRotation().distance(Rotation2d.fromDegrees(0.0))))
			return 180.0;
		return 0.0;
	}
	
	/** Sets MotionMagic targets for the drive motors */
	public void setPositionTarget(double directionDegrees, double magnitudeInches){
		setState(ControlState.POSITION);
		modules.forEach((m) -> m.setModuleAngle(directionDegrees));
		modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
	}
	
	/** Locks drive motors in place with MotionMagic */
	public void lockDrivePosition(){
		System.out.println("LOCKING MODULE POSITION");
		setState(ControlState.POSITION);
		isDriveLocked = false;
		//modules.forEach((m) -> m.setDrivePositionTarget(0.0));
	}

	public void atomicLockDrivePosition() {
		setState(ControlState.POSITION);
		modules.get(0).setModuleAngle(-45.0);
		modules.get(1).setModuleAngle(45.0);
		modules.get(2).setModuleAngle(-45.0);
		modules.get(3).setModuleAngle(45.0);
		modules.forEach((m) -> m.setDrivePositionTarget(0.0));
		isDriveLocked = false;
	}

	public void zukLockDrivePosition() {
		setState(ControlState.POSITION);
		modules.get(0).setModuleAngle(45.0);
		modules.get(1).setModuleAngle(-45.0);
		modules.get(2).setModuleAngle(45.0);
		modules.get(3).setModuleAngle(-45.0);
		isDriveLocked = false;
		//modules.forEach((m) -> m.setDrivePositionTarget(0.0));
	}
	
	/** Puts drive motors into closed-loop velocity mode */
	public void setVelocity(Rotation2d direction, double velocityInchesPerSecond){
		setState(ControlState.VELOCITY);
		modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
		modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
	}
	
	/** Configures each module to match its assigned vector */
	public void setDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
			}
		}
	}
	
	public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setDriveOpenLoop(-percentOutputOverride);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setDriveOpenLoop(percentOutputOverride);
			}
		}
	}
	
	
	/** Configures each module to match its assigned vector, but puts the drive motors into closed-loop velocity mode */
	public void setVelocityDriveOutput(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(driveVectors.get(i).norm() * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}
	
	public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverride){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-velocityOverride);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
				modules.get(i).setVelocitySetpoint(velocityOverride);
			}
		}
	}

	public void setVelocityDriveOutput(DriveSignal driveSignal, double velocityOverride) {
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-velocityOverride * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
				modules.get(i).setVelocitySetpoint(velocityOverride * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}

	public void setVelocityDriveOutput(DriveSignal driveSignal) {
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
				modules.get(i).setVelocitySetpoint(-driveSignal.getWheelSpeeds()[i] * Constants.kSwerveMaxSpeedInchesPerSecond);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
				modules.get(i).setVelocitySetpoint(driveSignal.getWheelSpeeds()[i] * Constants.kSwerveMaxSpeedInchesPerSecond);
			}
		}
	}
	
	/** Sets only module angles to match their assigned vectors */
	public void setModuleAngles(List<Translation2d> driveVectors){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveVectors.get(i).direction(), modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
			}else{
				modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
			}
		}
	}

	public void setModuleAngles(DriveSignal driveSignal){
		for(int i=0; i<modules.size(); i++){
			if(Util.shouldReverse(driveSignal.getWheelAzimuths()[i], modules.get(i).getModuleAngle())){
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees() + 180.0);
			}else{
				modules.get(i).setModuleAngle(driveSignal.getWheelAzimuths()[i].getDegrees());
			}
		}
	}
	
	/** Increases each module's rotational power cap for the beginning of auto */
	public void set10VoltRotationMode(boolean tenVolts){
		modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
	}
	
	/**
	* @return Whether or not at least one module has reached its MotionMagic setpoint
	*/
	public boolean positionOnTarget(){
		boolean onTarget = false;
		for(SwerveDriveModule m : modules){
			onTarget |= m.drivePositionOnTarget();
		}
		return onTarget;
	}
	
	/**
	* @return Whether or not all modules have reached their angle setpoints
	*/
	public boolean moduleAnglesOnTarget(){
		boolean onTarget = true;
		for(SwerveDriveModule m : modules){
			onTarget &= m.angleOnTarget();
		}
		return onTarget;
	}
	
	/**
	* Sets a trajectory for the robot to follow
	* @param trajectory 
	* @param targetHeading Heading that the robot will rotate to during its path following
	* @param rotationScalar Scalar to increase or decrease the robot's rotation speed
	* @param followingCenter The point (relative to the robot) that will follow the trajectory
	*/
	public void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
	double rotationScalar, Translation2d followingCenter){
		hasStartedFollowing = false;
		hasFinishedPath = false;
		moduleConfigRequested = false;
		motionPlanner.reset();
		motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
		motionPlanner.setFollowingCenter(followingCenter);
		inverseKinematics.setCenterOfRotation(followingCenter);
		setAbsolutePathHeading(targetHeading);
		this.rotationScalar = rotationScalar;
		trajectoryStartTime = Timer.getFPGATimestamp();
		setState(ControlState.TRAJECTORY);
	}
	
	public void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double targetHeading,
	double rotationScalar){
		setTrajectory(trajectory, targetHeading, rotationScalar, Translation2d.identity());
	}
	
	public void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading){
		setRobotCentricTrajectory(relativeEndPos, targetHeading, 45.0);
	}
	
	public void setRobotCentricTrajectory(Translation2d relativeEndPos, double targetHeading, double defaultVel){
		modulesReady = true;
		Translation2d endPos = pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation();
		Rotation2d startHeading = endPos.translateBy(pose.getTranslation().inverse()).direction();
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(pose.getTranslation(), startHeading));	
		waypoints.add(new Pose2d(pose.transformBy(Pose2d.fromTranslation(relativeEndPos)).getTranslation(), startHeading));
		Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
		double heading = Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), targetHeading);
		setTrajectory(trajectory, heading, 1.0);
	}

	public synchronized void setFieldCentricTrajectory(Translation2d relativeEndPos, double targetHeading, double defaultVel) {
		modulesReady = true;
		Translation2d endPos = pose.getTranslation().translateBy(relativeEndPos);
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(pose.getTranslation(), relativeEndPos.direction()));
		waypoints.add(new Pose2d(endPos, relativeEndPos.direction()));
		Trajectory<TimedState<Pose2dWithCurvature>> trajectory = generator.generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 60.0, 60.0, 9.0, defaultVel, 1);
		double heading = Util.placeInAppropriate0To360Scope(pose.getRotation().getUnboundedDegrees(), targetHeading);
		setTrajectory(trajectory, heading, 1.0);
	}
	
	// Vision PID (new, simpler vision tracking system)
	public void startVisionPID(Translation2d endTranslation, Rotation2d approachAngle, Rotation2d targetHeading) {
		Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
		if(aim.isPresent()) {
			useFixedVisionOrientation = (approachAngle != null);
			Optional<Pose2d> scoringPos = robotState.getRobotScoringPosition(aim, useFixedVisionOrientation ? approachAngle : aim.get().getTurretToGoal().direction(), endTranslation);
			if(scoringPos.isPresent()) {
				lateralPID.setSetpoint(0.0);
				forwardPID.setSetpoint(0.0);
				fixedVisionOrientation = (useFixedVisionOrientation ? approachAngle : aim.get().getTurretToGoal().direction());
				if (useFixedVisionOrientation)
					visionApproachAngle = approachAngle;
				lastVisionEndTranslation = endTranslation;
				visionPIDTarget = scoringPos.get();
				visionTargetAcquired = false;
				visionTargetReached = /*true*/false;
				rotationScalar = 0.5;
				setPathHeading(targetHeading.getDegrees());
				setState(ControlState.VISION_PID);
			} else {
				System.out.println("No target detected");
				visionTargetAcquired = false;
			}
		} else {
			System.out.println("No target detected");
			visionTargetAcquired = false;
		}
	}

	public void startVisionPID(Translation2d endTranslation, Rotation2d approachAngle) {
		startVisionPID(endTranslation, approachAngle, pose.getRotation());
	}
	
	public void startVisionPID(Translation2d endTranslation) {
		startVisionPID(endTranslation, null, pose.getRotation());
	}
	
	Translation2d updateVisionPID(double dt) {
		Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
		if(aim.isPresent() /*&& visionTargetReached*/) {
			if(!useFixedVisionOrientation) 
				fixedVisionOrientation = aim.get().getTurretToGoal().direction();
			else
				fixedVisionOrientation = visionApproachAngle;
			Optional<Pose2d> scoringPos = robotState.getRobotScoringPosition(aim, fixedVisionOrientation, lastVisionEndTranslation);
			if(scoringPos.isPresent()) {
				visionPIDTarget = scoringPos.get();
				visionTargetAcquired = true;
				//visionTargetReached = false;
			}
		} 
		
		if(visionTargetAcquired) {
			Translation2d adjustedTarget = visionPIDTarget.getTranslation().rotateBy(fixedVisionOrientation.inverse());
			Translation2d adjustedPose = pose.transformBy(RobotState.kVehicleToTurretFixed).getTranslation().rotateBy(fixedVisionOrientation.inverse());
			Translation2d error = adjustedTarget.translateBy(adjustedPose.inverse());
			Translation2d output = new Translation2d(-forwardPID.calculate(error.x(), dt), -lateralPID.calculate(error.y(), dt));
			double magnitude = Util.deadBand(output.norm(), 0.01);
			output = Translation2d.fromPolar(output.direction(), magnitude);
			if(output.norm() > Constants.kVisionPIDOutputPercent) {
				//normalize the output vector
				output = Translation2d.fromPolar(output.direction(), Constants.kVisionPIDOutputPercent);
			}
			output = output.rotateBy(fixedVisionOrientation);
			visionTargetHeading = output.direction();
			if (error.norm() <= Constants.kDistanceToTargetTolerance) {
				visionTargetReached = true;
				//output = new Translation2d();
				//setState(ControlState.NEUTRAL);
				lockDrivePosition();
			}
			System.out.println("Vision PID output vector: " + output.toString() + "Error Norm: " + error.norm());
			SmartDashboard.putNumber("Vision Output Vector Angle", output.direction().getDegrees());
			return output;
		} else {
			System.out.println("No target detected");
			return new Translation2d();
		}
	}
	
	/****************************************************/
	/* Vector Fields */
	public synchronized void setVectorField(VectorField vf_) {
		vf = vf_;
		setState(ControlState.VECTORIZED);
	}
	
	/** Determines which wheels the robot should rotate about in order to perform an evasive maneuver */
	public synchronized void determineEvasionWheels(){
		Translation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
		List<Translation2d> wheels = Constants.kModulePositions;
		clockwiseCenter = wheels.get(0);
		counterClockwiseCenter = wheels.get(wheels.size()-1);
		for(int i = 0; i < wheels.size()-1; i++) {
			Translation2d cw = wheels.get(i);
			Translation2d ccw = wheels.get(i+1);
			if(here.isWithinAngle(cw,ccw)) {
				clockwiseCenter = ccw;
				counterClockwiseCenter = cw;
			}
		}
	}
	
	/** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveDriveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = 100.0;
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for(SwerveDriveModule m : positionModules){
			double deviance = Math.abs(distances[m.moduleID] - averageDistance);
			if(deviance < minDeviance){
				minDeviance = deviance;
				minDevianceIndex = m.moduleID;
			}
			if(deviance <= 0.01){
				modulesToUse.add(m);
			}
		}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		
		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
	
	/** Playing around with different methods of odometry. This will require the use of all four modules, however. */
	public synchronized void alternatePoseUpdate(){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = pigeon.getYaw();
		
		double[][] distances = new double[4][2];
		for(SwerveDriveModule m : modules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
			distances[m.moduleID][0] = m.moduleID;
			distances[m.moduleID][1] = distance;
		}
		
		Arrays.sort(distances, new java.util.Comparator<double[]>() {
			public int compare(double[] a, double[] b) {
				return Double.compare(a[1], b[1]);
			}
		});
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		double firstDifference = distances[1][1] - distances[0][1];
		double secondDifference = distances[2][1] - distances[1][1];
		double thirdDifference = distances[3][1] - distances[2][1];
		if(secondDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
		}else if(thirdDifference > (1.5 * firstDifference)){
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
		}else{
			modulesToUse.add(modules.get((int)distances[0][0]));
			modulesToUse.add(modules.get((int)distances[1][0]));
			modulesToUse.add(modules.get((int)distances[2][0]));
			modulesToUse.add(modules.get((int)distances[3][0]));
		}
		
		SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
		distanceTraveled += deltaPos;
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
	
	boolean isDriveLocked = false;
	/** Called every cycle to update the swerve based on its control state */
	public synchronized void updateControlCycle(double timestamp){
		double rotationCorrection = headingController.updateRotationCorrection(pose.getRotation().getUnboundedDegrees(), timestamp);
		switch(currentState){
			case MANUAL:
			if(evading && evadingToggled){
				determineEvasionWheels();
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
				evadingToggled = false;
			}else if(evading){
				double sign = Math.signum(rotationalInput);
				if(sign == 1.0){
					inverseKinematics.setCenterOfRotation(clockwiseCenter);
				}else if(sign == -1.0){
					inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
				}
			}else if(evadingToggled){
				inverseKinematics.setCenterOfRotation(Translation2d.identity());
				evadingToggled = false;
			}
			if(translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0){
				if(lastDriveVector.equals(rotationalVector)){
					stop();
				}else{
					setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector,
					rotationCorrection, pose, robotCentric), 0.0);
				}
			}else{
				setDriveOutput(inverseKinematics.updateDriveVectors(translationalVector,
				rotationalInput + rotationCorrection, pose, robotCentric));
			}
			break;
			case POSITION:
				if (moduleAnglesOnTarget() && !isDriveLocked) {
					modules.forEach((m) -> m.setDrivePositionTarget(0.0));
					this.isDriveLocked = true;
				}
			break;
			case ROTATION:
			setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), Util.deadBand(rotationCorrection, 0.1), pose, false));
			break;
			case VECTORIZED:
			Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
			SmartDashboard.putNumber("Vector Direction", outputVectorV.direction().getDegrees());
			SmartDashboard.putNumber("Vector Magnitude", outputVectorV.norm());
			//			System.out.println(outputVector.x()+" "+outputVector.y());
			setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, getPose(), false));
			break;
			case TRAJECTORY:
			if(!motionPlanner.isDone()){
				Translation2d driveVector = motionPlanner.update(timestamp, pose);
				
				if(modulesReady){
					if(!hasStartedFollowing){
						if(moduleConfigRequested){
							zeroSensors(startingPose);
							System.out.println("Position reset for auto");
						}
						hasStartedFollowing = true;
					}
					double rotationInput = Util.deadBand(Util.limit(rotationCorrection*rotationScalar*driveVector.norm(), motionPlanner.getMaxRotationSpeed()), 0.01);
					if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
						driveVector = lastTrajectoryVector;
						/*setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
						rotationInput, pose, false), 0.0);*/
						setVelocityDriveOutput(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true), 0.0);
					}else{
						/*setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, 
						rotationInput, pose, false));*/
						setVelocityDriveOutput(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), rotationInput, true));
					}
				}else if(!moduleConfigRequested){
					//set10VoltRotationMode(true);
					/*setModuleAngles(inverseKinematics.updateDriveVectors(driveVector, 
					0.0, pose, false));*/
					setModuleAngles(Kinematics.inverseKinematics(driveVector.x(), driveVector.y(), 0.0, true));
					moduleConfigRequested = true;
				}
				
				if(moduleAnglesOnTarget() && !modulesReady){
					set10VoltRotationMode(false);
					modules.forEach((m) -> m.resetLastEncoderReading());
					modulesReady = true;
					System.out.println("Modules Ready");
				}
				
				lastTrajectoryVector = driveVector;
			}else{
				if(!hasFinishedPath){ 
					System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
					hasFinishedPath = true;
					if(alwaysConfigureModules) requireModuleConfiguration();
				}
			}
			break;
			case VISION_PID:
			Translation2d driveVector = updateVisionPID(timestamp - lastUpdateTimestamp);
			if(Util.epsilonEquals(driveVector.norm(), 0.0, Constants.kEpsilon)){
				driveVector = lastDriveVector;
				setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false), 0.0);
			}else{
				setVelocityDriveOutput(inverseKinematics.updateDriveVectors(driveVector, Util.deadBand(rotationCorrection*rotationScalar, 0.01), pose, false));
			}
			lastDriveVector = driveVector;
			break;
			case VELOCITY:
			break;
			case NEUTRAL:
			stop();
			break;
			case DISABLED:
			
			break;
			default:
			break;
		}
	}
	
	private final Loop loop = new Loop(){
		
		@Override
		public void onStart(double timestamp) {
			translationalVector = new Translation2d();
			lastDriveVector = rotationalVector;
			rotationalInput = 0;
			resetAveragedDirection();
			headingController.temporarilyDisable();
			stop();
			outputWpiPose = false;
			lastUpdateTimestamp = timestamp;
		}
		
		@Override
		public void onLoop(double timestamp) {
			synchronized(Swerve.this){
				if(modulesReady || (getState() != ControlState.TRAJECTORY)){
					//updatePose(timestamp);
					//alternatePoseUpdate();
					//wpiPose = odometry.update(pose.getRotation(), getModuleStates());
					pose = odometry.update(pigeon.getYaw(), getModuleStates());
					velocity = odometry.getVelocity();
					//pose = robotState.getLatestFieldToVehicle().getValue();
				}
				updateControlCycle(timestamp);
				lastUpdateTimestamp = timestamp;
			}
		}
		
		@Override
		public void onStop(double timestamp) {
			translationalVector = new Translation2d();
			rotationalInput = 0;
			outputWpiPose = false;
			stop();
		}
		
	};
	
	public Request visionPIDRequest(Translation2d endTranslation, Rotation2d approachAngle, double cutoffDistance) {
		return new Request(){
			
			@Override
			public void act() {
				if (!isTracking()) {
					visionCutoffDistance = cutoffDistance;
					startVisionPID(endTranslation, approachAngle);
					System.out.println("Vision Request started");
				}
			}
			
			@Override
			public boolean isFinished(){
				return getState() == ControlState.VISION_PID && (robotState.distanceToTarget() < visionCutoffDistance);
			}
			
		};
	}

	public Request visionPIDRequest(Translation2d endTranslation, Rotation2d approachAngle) {
		return new Request(){
		
			@Override
			public void act() {
				if (!isTracking()) {
					startVisionPID(endTranslation, approachAngle);
					System.out.println("Vision Request Started");
				}
			}

			@Override
			public boolean isFinished() {
				if (visionTargetReached) {
					//setState(ControlState.NEUTRAL);
					DriverStation.reportError("Swerve Vision PID Request Finished", false);
					return true;
				}
				return false;
			}

		};
	}

	public Request visionPIDRequest(Translation2d endTranslation) {
		return new Request(){
		
			@Override
			public void act() {
				if (!isTracking()) {
					startVisionPID(endTranslation);
					System.out.println("Vision Request Started");
				}
			}

			@Override
			public boolean isFinished() {
				if (visionTargetReached) {
					//setState(ControlState.NEUTRAL);
					DriverStation.reportError("Swerve Vision PID Request Finished", false);
					return true;
				}
				return false;
			}

		};
		
	}
	
	public Request robotCentricTrajectoryRequest(Translation2d relativeEndPos, double targetHeading, double defaultVel){
		return new Request(){
			
			@Override
			public void act() {
				setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
			@Override
			public boolean isFinished(){
				return (getState() == ControlState.TRAJECTORY && motionPlanner.isDone()) || getState() == ControlState.MANUAL;
			}
			
		};
	}
	
	public Request startRobotCentricTrajectoryRequest(Translation2d relativeEndPos, double targetHeading, double defaultVel){
		return new Request(){
			
			@Override
			public void act() {
				setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
		};
	}

	public Request fieldCentricTrajectoryRequest(Translation2d relativeEndPos, double targetHeading, double defaultVel) {
		return new Request(){
			
			@Override
			public void act() {
				setFieldCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
			}
			
			@Override
			public boolean isFinished(){
				return (getState() == ControlState.TRAJECTORY && motionPlanner.isDone()) || getState() == ControlState.MANUAL;
			}
			
		};
	}
	
	public Request openLoopRequest(Translation2d input, double rotation){
		return new Request(){
			
			@Override
			public void act() {
				setState(ControlState.MANUAL);
				sendInput(input.x(), input.y(), rotation, false, false);
			}
			
		};
	}
	
	public Request velocityRequest(Rotation2d direction, double magnitude){
		return new Request(){
			
			@Override
			public void act() {
				setVelocity(direction, magnitude);
			}
			
		};
	}

	public Request lockDrivePositionRequest() {
		return new Request(){
		
			@Override
			public void act() {
				lockDrivePosition();
			}
			
		};
	}
	public Request setDriveMaxPowerRequest(double power) {
		return new Request() {

			@Override
			public void act() {
				setMaxSpeed(power);
			}
		};
	}
	
	public void setNominalDriveOutput(double voltage){
		modules.forEach((m) -> m.setNominalDriveOutput(voltage));
	}
	
	/** Sets the maximum rotation speed opf the modules, based on the robot's velocity */
	public void setMaxRotationSpeed(){
		double currentDriveSpeed = translationalVector.norm() * Constants.kSwerveMaxSpeedInchesPerSecond;
		double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed / 
		((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
		modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
	}
	
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.size()]; 
		for(int i = 0; i < modules.size(); i++) {
			states[i] = modules.get(i).getState();
		}
		return states;
	}

	public double[] getModuleVelocities() {
		double[] velocities = new double[modules.size()];
		SwerveModuleState[] states = getModuleStates();
		for (int i = 0; i < modules.size(); i++) {
			velocities[i] = states[i].speedMetersPerSecond;
		}
		return velocities;
	}

	public Rotation2d[] getModuleAngles() {
		Rotation2d[] angles = new Rotation2d[modules.size()];
		SwerveModuleState[] states = getModuleStates();
		for (int i = 0; i < modules.size(); i++) {
			angles[i] = states[i].angle;
		}
		return angles;
	}

	public Rotation2d getHeading() {
		return pigeon.getYaw();
	}

	public void zeroModuleAngles() {
		modules.forEach((m) -> m.resetRotationToAbsolute());
	}

	public void setRotationMotorZeroed(boolean isZeroed) {
		modules.forEach((m) -> m.setRotationMotorZeroed(isZeroed));
	}
	
	@Override
	public void readPeriodicInputs() {
		modules.forEach((m) -> m.readPeriodicInputs());
	}
	
	@Override
	public void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
	}
	
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	/** Puts all rotation and drive motors into open-loop mode */
	public synchronized void disable(){
		modules.forEach((m) -> m.disable());
		setState(ControlState.DISABLED);
	}
	
	@Override
	public void stop() {
		setState(ControlState.NEUTRAL);
		modules.forEach((m) -> m.stop());
	}
	
	@Override
	public void zeroSensors() {
		zeroSensors(Constants.kRobotStartingPose);
	}
	
	/** Zeroes the drive motors, and sets the robot's internal position and heading to match that of the fed pose */
	public void zeroSensors(Pose2d startingPose){
		pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
		modules.forEach((m) -> m.zeroSensors(startingPose));
		pose = startingPose;
		odometry.resetPosition(startingPose, startingPose.getRotation());
		robotState.reset(Timer.getFPGATimestamp(), startingPose, Rotation2d.fromDegrees(Turret.getInstance().getAngle()));
		distanceTraveled = 0;
	}
	
	public synchronized void resetPosition(Pose2d newPose){
		pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(pose, pose.getRotation());
		robotState.reset(Timer.getFPGATimestamp(), pose, Rotation2d.fromDegrees(Turret.getInstance().getAngle()));
		distanceTraveled = 0;
	}
	
	public synchronized void setXCoordinate(double x){
		pose.getTranslation().setX(x);
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(pose, pose.getRotation());
		System.out.println("X coordinate reset to: " + pose.getTranslation().x());
	}
	
	public synchronized void setYCoordinate(double y){
		pose.getTranslation().setY(y);
		modules.forEach((m) -> m.zeroSensors(pose));
		odometry.resetPosition(pose, pose.getRotation());
		System.out.println("Y coordinate reset to: " + pose.getTranslation().y());
	}
	
	@Override
	public void outputTelemetry() {
		modules.forEach((m) -> m.outputTelemetry());
		SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
		SmartDashboard.putNumber("Robot Heading", pose.getRotation().getUnboundedDegrees());
		// testing the wpi odometry
		if(outputWpiPose)
		SmartDashboard.putNumberArray("Path Pose", new double[]{wpiPose.getTranslation().x(), wpiPose.getTranslation().y(), wpiPose.getRotation().getUnboundedDegrees()});
		if(Settings.debugSwerve()){
			SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
			SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
			SmartDashboard.putString("Heading Controller", headingController.getState().toString());
			SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
			SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
			SmartDashboard.putString("Robot Velocity", velocity.toString());
			SmartDashboard.putString("Swerve State", currentState.toString());
			SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
			SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());
		}
	}
}
