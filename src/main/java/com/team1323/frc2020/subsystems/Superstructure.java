package com.team1323.frc2020.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.LimelightProcessor;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.LambdaRequest;
import com.team1323.frc2020.subsystems.requests.ParallelRequest;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.subsystems.requests.SequentialRequest;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	private Compressor compressor;
	
	public Swerve swerve;
	public Intake intake;
    public Wrist wrist;
    public BallSplitter ballSplitter;
    public BallFeeder ballFeeder;
    public Column column;
    public Turret turret;
    public Shooter shooter;
	public MotorizedHood motorizedHood;
	public RobotState robotState;
	
	public Superstructure(){
		//compressor = new Compressor(Ports.PCM, PneumaticsModuleType.CTREPCM);
		
		swerve = Swerve.getInstance();
		intake = Intake.getInstance();
        wrist = Wrist.getInstance();
        ballSplitter = BallSplitter.getInstance();
        ballFeeder = BallFeeder.getInstance();
        column = Column.getInstance();
        turret = Turret.getInstance();
        shooter = Shooter.getInstance();
		motorizedHood = MotorizedHood.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}

	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}

	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			if(newRequest && activeRequest != null) {
				activeRequest.act();
				newRequest = false;
			} 

			if(activeRequest == null) {
				if(queuedRequests.isEmpty()) {
					allRequestsCompleted = true;
				} else {
					setActiveRequest(queuedRequests.remove(0));
				}
			} else if(activeRequest.isFinished()) {
				activeRequest = null;
			}
		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};
	
	public void enableCompressor(boolean enable){
		/*if (enable) {
			compressor.enableDigital();
		} else {
			compressor.disable();
		}*/
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
	}
	
	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}

		};
	}


	private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
    }

	///// States /////

	public void neutralState() {
		request(new ParallelRequest(
		));
	}

	/**
	 * Moves the robot into the specified distance from the target
	 * @param translationFromTarget Negative distance means in front of the target and positive means behind the target
	 */
	public void moveIntoRangeState(Translation2d translationFromTarget) {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				swerve.visionPIDRequest(translationFromTarget)
			)
		);
	}

	/**
	 * Moves the robot into the specified distance from the target and with final orientation
	 * @param translationFromTarget Negative distance means in front of the target and positive means behind the target
	 * @param approachAngle The robot will attempt to approach the target along a line extending from the target. The
	 * angle of that line (relative to the field) is specified by approachAngle
	 */
	public void moveIntoRangeState(Translation2d translationFromTarget, Rotation2d approachAngle) {
		request(
			new SequentialRequest(
				new LambdaRequest(() -> LimelightProcessor.getInstance().ledOn(true)),
				swerve.visionPIDRequest(translationFromTarget, approachAngle)
			)
		);
	}
		
	public void intakeState() {
		request(
			new SequentialRequest(
				new ParallelRequest(
					ballFeeder.stateRequest(BallFeeder.State.DETECT),
					intake.stateRequest(Intake.ControlState.INTAKE),
					wrist.setWristIntakeRequest(),
					column.stateRequest(Column.ControlState.INDEX_BALLS)
				)		
			)
		);
	}

	public void postIntakeState() {
		request(
			new ParallelRequest(
				wrist.setWristAngleRequest(Constants.Wrist.kBallDebouncerAngle),
				intake.stateRequest(Intake.ControlState.OFF),
				ballFeeder.stateRequest(BallFeeder.State.OFF)
			)
		);
	}

	public void autoRotateEjectState(boolean enabled) {
		request(
			new ParallelRequest(
				ballSplitter.swerveAutoRotateRequest(enabled)
			)
		);
	}

	public void manualShotState(double shooterRPM, double rawHoodAngle) {
		request(
			new SequentialRequest(
				new ParallelRequest(
					motorizedHood.setAngleRequest(Constants.MotorizedHood.kMinControlAngle + rawHoodAngle),
					turret.robotStateVisionRequest(),
					shooter.velocityRequest(shooterRPM) //2200 - Mid //2100 - Close //2400
				),
				intake.stateRequest(Intake.ControlState.INTAKE)
				//column.stateRequest(Column.ControlState.FEED_BALLS)
			)
		);
	}

	public void visionTrackState() {
		request(
			new ParallelRequest(
				motorizedHood.visionRequest(),
				turret.startVisionRequest(),
				shooter.visionVelocityRequest()
			)
		);
	}

	public void visionShotState() {
		request(
			new SequentialRequest(
				new ParallelRequest(
					motorizedHood.visionRequest(),
					swerve.setDriveMaxPowerRequest(0.75),
					turret.robotStateVisionRequest(),
					shooter.visionVelocityRequest()
				),
				turret.robotStateVisionRequest(),
				intake.stateRequest(Intake.ControlState.INTAKE),
				column.stateRequest(Column.ControlState.FEED_BALLS)
			)
		);
	}

	public void postShotState() {
		request(
			new ParallelRequest(
				column.stateRequest(Column.ControlState.OFF),
				ballFeeder.stateRequest(BallFeeder.State.DETECT),
				swerve.setDriveMaxPowerRequest(1.0),
				//motorizedHood.setAngleRequest(Constants.MotorizedHood.kMinControlAngle),
				shooter.openLoopRequest(0.25),
				intake.stateRequest(Intake.ControlState.OFF),
				new LambdaRequest(()-> {
					Optional<Pose2d> newRobotPose = RobotState.getInstance().getEstimatedRobotPosition();
					if (newRobotPose.isPresent()) {
						swerve.resetPosition(newRobotPose.get());
					} else {
						System.out.println("Vision target not present when trying to update robot position!");
					}
				}),
				new LambdaRequest(() -> {
					if (turret.getState() == Turret.ControlState.ROBOT_STATE_VISION) {
						turret.startVision();
					} else {
						turret.lockAngle();
					}
				})
			)
		);
	}

	public void intakeAndTrackState() {//Used for auto
		request(
			new ParallelRequest(
				new SequentialRequest(
					new ParallelRequest(
						motorizedHood.visionRequest(),
						turret.startVisionRequest(),
						shooter.visionVelocityRequest()
					)
				),
				new SequentialRequest(
					ballFeeder.intakeFeedRequest(),
					wrist.setWristIntakeRequest(),
					intake.stateRequest(Intake.ControlState.INTAKE),
					new LambdaRequest(()-> column.setState(Column.ControlState.INDEX_BALLS))
				)
			)
		);
	}
	
	public void reverseAllSubsystems() {
		request(
			new ParallelRequest(
				intake.stateRequest(Intake.ControlState.EJECT),
				ballFeeder.openLoopRequest(0.5),
				column.stateRequest(Column.ControlState.EJECT)
			)
		);
	}
	public void disableState() {
		request(
			new ParallelRequest(
				intake.stateRequest(Intake.ControlState.OFF),
				ballFeeder.stateRequest(BallFeeder.State.OFF),
				ballSplitter.stateRequest(BallSplitter.ControlState.OFF),
				column.stateRequest(Column.ControlState.OFF),
				shooter.openLoopRequest(0.0)
			)
		);
	}
}
