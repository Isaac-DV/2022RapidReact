/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2020;

import java.util.Set;

import com.team1323.frc2020.auto.AutoModeBase;
import com.team1323.frc2020.auto.AutoModeExecuter;
import com.team1323.frc2020.auto.SmartDashboardInteractions;
import com.team1323.frc2020.auto.modes.FiveBallOneEjectMode;
import com.team1323.frc2020.auto.modes.ThreeBallPoachBlueAllianceMode;
import com.team1323.frc2020.auto.modes.TwoBallBackHubHideMode;
import com.team1323.frc2020.auto.modes.TwoBallCloseHideMode;
import com.team1323.frc2020.auto.modes.TwoBallTwoEjectMode;
import com.team1323.frc2020.loops.LimelightProcessor;
import com.team1323.frc2020.loops.Looper;
import com.team1323.frc2020.loops.QuinticPathTransmitter;
import com.team1323.frc2020.loops.RobotStateEstimator;
import com.team1323.frc2020.subsystems.DoubleTelescopes;
import com.team1323.frc2020.subsystems.LEDs;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.SubsystemManager;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team1323.frc2020.subsystems.Turret;
import com.team1323.frc2020.subsystems.Wrist;
import com.team1323.frc2020.subsystems.LEDs.LEDColors;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.trajectory.TrajectoryGenerator;
// import com.wpilib.TimedRobot; // modified TimedRobot to remove watchdog error printing

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	private LEDs leds;
	private Superstructure s;
	private SubsystemManager subsystems;

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();

	private DriverControls driverControls;	

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driverControls = DriverControls.getInstance();
		
	
		//pigeon = Pigeon.getInstance();
		leds = LEDs.getInstance();
		s = Superstructure.getInstance();
		subsystems = driverControls.getSubsystems();

		Logger.clearLog();

		enabledLooper.register(driverControls);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		s.swerve.zeroSensors(new Pose2d());
		s.swerve.stop();

		smartDashboardInteractions.initWithDefaults();

		Settings.initializeToggles();

		generator.generateTrajectories();

		AutoModeBase auto = new ThreeBallPoachBlueAllianceMode();
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));

		/*UsbCamera usbCam = new UsbCamera("IntakeCam", 0);
		usbCam.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
		new MjpegServer("cameraServer", 1182).setSource(usbCam);
		//usbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);*/
		
		/*UsbCamera usbCam = CameraServer.startAutomaticCapture(0);
		VideoSink server = CameraServer.getServer();
		usbCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
		usbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		server.setSource(usbCam);*/
	}

	@Override
	public void robotPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", DriverStation.isEnabled());
		SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			driverControls.setAutoMode(true);
			Wrist.getInstance().zeroWrist();
			Turret.getInstance().zeroTurret();
			MotorizedHood.getInstance().zeroHood();
			disabledLooper.stop();
			enabledLooper.start();

			SmartDashboard.putBoolean("Auto", true);

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			driverControls.setAutoMode(false);
			disabledLooper.stop();
			enabledLooper.start();
			Wrist.getInstance().zeroWrist();
			Turret.getInstance().zeroTurret();
			MotorizedHood.getInstance().zeroHood();
			SmartDashboard.putBoolean("Auto", false);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			enabledLooper.outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
			leds.configLEDs(LEDColors.GREEN);
			DoubleTelescopes.getInstance().setTelescopesBrakeMode();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			disabledLooper.outputToSmartDashboard();
			smartDashboardInteractions.output();
			Settings.update();
			s.swerve.zeroModuleAngles();
			Wrist.getInstance().resetToAbsolutePosition();
			Turret.getInstance().resetToAbsolutePosition();
			if (subsystems.haveEmergency()) {
				leds.configLEDs(LEDColors.RED);
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

	public void printStackTrace() {
		Set<Thread> threads = Thread.getAllStackTraces().keySet();
	
		for (Thread thread : threads) {
		  System.out.println("Thread name: " + thread.getName());
		  System.out.println("Thread state: " + thread.getState());
	
		  StackTraceElement[] stackTraceElements = thread.getStackTrace();
		  for (StackTraceElement stackTraceElement : stackTraceElements) {
			System.out.println("\t" + stackTraceElement);
		  }
		  System.out.println("\n");
		}
	  }

}
