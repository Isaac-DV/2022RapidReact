/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2020;

import java.util.Arrays;

import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.BallEjector;
import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.BallSplitter;
import com.team1323.frc2020.subsystems.Column;
import com.team1323.frc2020.subsystems.Elevator;
import com.team1323.frc2020.subsystems.Intake;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.Shooter;
import com.team1323.frc2020.subsystems.SubsystemManager;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team1323.frc2020.subsystems.Swerve;
import com.team1323.frc2020.subsystems.Turret;
import com.team1323.frc2020.subsystems.Wrist;
import com.team1323.io.Xbox;
import com.team254.lib.geometry.Pose2d;

/**
 * A class to assign controller inputs to robot actions
 */
public class DriverControls implements Loop {

    private static DriverControls instance = null;

    public static DriverControls getInstance() {
        if (instance == null)
            instance = new DriverControls();
        return instance;
    }

	Xbox driver, coDriver, testController;

    private Swerve swerve;
    private Intake intake;
    private Wrist wrist;
    private BallSplitter ballSplitter;
    private BallEjector ballEjector;
    private BallFeeder ballFeeder;
    private Column column;
    private Turret turret;
    private MotorizedHood motorizedHood;
    private Shooter shooter;
    private Elevator elevator;
    private Superstructure s;

    private SubsystemManager subsystems;
    public SubsystemManager getSubsystems(){ return subsystems; }

    private final boolean oneControllerMode = false;
    private boolean robotCentric = false;
        
    private boolean inAuto = true;
    public void setAutoMode(boolean auto) {
        inAuto = auto;
    }

    public boolean getInAuto() {
        return inAuto;
    }

    public DriverControls() {
        driver = new Xbox(0);
		coDriver = new Xbox(1);
        testController = new Xbox(2);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.25);
		coDriver.rightBumper.setLongPressDuration(1.0);

        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        wrist = Wrist.getInstance();
        ballSplitter = BallSplitter.getInstance();
        ballEjector = BallEjector.getInstance();
        ballFeeder = BallFeeder.getInstance();
        column = Column.getInstance();
        turret = Turret.getInstance();
        motorizedHood = MotorizedHood.getInstance();
        shooter = Shooter.getInstance();
        elevator = Elevator.getInstance();  

        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(
				Arrays.asList(swerve, intake, wrist, ballSplitter, ballEjector, ballFeeder, column, turret,
                    motorizedHood, shooter, elevator, s));
    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.setNominalDriveOutput(0.0);
            swerve.requireModuleConfiguration();
            s.enableCompressor(false);
        } else {
            s.enableCompressor(true);
            swerve.setNominalDriveOutput(0.0);
            swerve.set10VoltRotationMode(false);
        }
        motorizedHood.setState(MotorizedHood.State.VISION);
        wrist.setWristLocked();
    }

    @Override
    public void onLoop(double timestamp) {
        RobotState.getInstance().outputToSmartDashboard();
        if(inAuto) {
            // Any auto-specific LED controls can go here
        } else {
            driver.update();
			coDriver.update();
            if(oneControllerMode) oneControllerMode();
            else twoControllerMode();
        }
    }

    @Override
    public void onStop(double timestamp) {
        subsystems.stop();
    }

    private void twoControllerMode() {
        double swerveYInput = driver.getLeftX();
        double swerveXInput = -driver.getLeftY();
        double swerveRotationInput = driver.getRightX();
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());
        
        if (driver.bButton.isBeingPressed())
            swerve.rotate(90);
        else if (driver.aButton.isBeingPressed()) 
            swerve.rotate(180);
        else if (driver.xButton.isBeingPressed())
            swerve.rotate(270);
        else if (driver.yButton.isBeingPressed())
            swerve.rotate(0.0);

        if (driver.startButton.isBeingPressed()) 
            swerve.setState(Swerve.ControlState.NEUTRAL);

        if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
        }
        if(driver.rightBumper.wasActivated()) {
            s.autoRotateEjectState(true);
        } else if(driver.rightBumper.wasReleased()) {
            s.autoRotateEjectState(false);
        }

        if(s.needsToNotifyDrivers()) {
            driver.rumble(1.0, 2.0);
            coDriver.rumble(1.0, 2.0);
        }

		////// Official Controls //////
        double coDriverRightX = coDriver.getRightX();
        double coDriverRightY = coDriver.getRightX();
        double coDriverLeftY = -coDriver.getLeftY();
        double coDriverLeftX = coDriver.getLeftX();

        /*if (coDriver.backButton.wasActivated()){
            s.neutralState();
            swerve.setState(Swerve.ControlState.NEUTRAL);
        }
        */
        
        if(coDriverLeftY != 0 || coDriverLeftX != 0) {
            turret.fieldRelativeManual(coDriverLeftX, coDriverLeftY);
        } else if(turret.getState() == Turret.ControlState.OPEN_LOOP) {
            turret.lockAngle();
        }
        /*
        if(coDriverLeftY != 0) {
            wrist.setOpenLoop(coDriverLeftY);
        } else if(wrist.getState() == Wrist.State.OPEN_LOOP) {
            wrist.setWristLockedAtCurrentAngle();
        }*/
        if(coDriverRightX != 0) {
            turret.setOpenLoop(coDriverRightX);
        } else if(turret.getState() == Turret.ControlState.OPEN_LOOP) {
            turret.lockAngle();
        }
        if(coDriver.POV0.wasActivated()) {
            motorizedHood.setAngle(Constants.MotorizedHood.kMaxControlAngle);
        } else if(coDriver.POV0.wasReleased()) {
            motorizedHood.setAngle(Constants.MotorizedHood.kMinControlAngle);
        }

        
        if(coDriver.aButton.wasActivated()) {
            intake.conformToState(Intake.ControlState.INTAKE);
            wrist.setWristAngle(Constants.Wrist.kIntakeAngle);
            ballFeeder.setState(BallFeeder.State.DETECT);
            if(column.getState() != Column.ControlState.FEED_BALLS) {
                column.setState(Column.ControlState.INDEX_BALLS);
            }
        } else if(coDriver.aButton.wasReleased()) {
            intake.conformToState(Intake.ControlState.OFF);
            wrist.setWristAngle(Constants.Wrist.kBallDebouncerAngle);
            ballFeeder.queueShutdown(true);
        }
        if(coDriver.bButton.longReleased() || coDriver.bButton.longPressed()) {
            wrist.setWristAngle(Constants.Wrist.kStowedAngle);
        }else if(coDriver.bButton.shortReleased()) {
            wrist.setWristAngle(Constants.Wrist.kLowestAngle);
        }
        if(coDriver.yButton.wasActivated()) {
            s.manualShotState(2950.0, 12.5);
        } else if(coDriver.yButton.wasReleased()) {
            s.postShotState();
        }
        if(coDriver.xButton.wasActivated()) {
            //s.manualShotState(1600.0, 5.0);
            //shooter.setOpenLoop(1.0);
            shooter.setVelocity(2000.0);
        } else if(coDriver.xButton.wasReleased()) {
            s.postShotState();
        }
        

        if(coDriver.rightBumper.wasActivated()) {
            column.setVelocity(6380.0 * 0.70);
        } else if(coDriver.rightBumper.wasReleased()) {
            column.conformToState(Column.ControlState.OFF);
        }
        if(coDriver.leftBumper.wasActivated()) {
            s.reverseAllSubsystems();
        } else if(coDriver.leftBumper.wasReleased()) {
            s.disableState();
        }
           

        if(coDriver.rightTrigger.wasActivated()) {
            s.visionShotState();
        } else if(coDriver.rightTrigger.wasReleased()) {
            s.postShotState();
        }
        if(coDriver.leftTrigger.wasActivated()) {
            turret.startVision();
        }

        if(coDriver.rightCenterClick.wasActivated()) {
            turret.setAngle(0.0);
        }


        
        if(coDriver.backButton.wasActivated()) {
            s.disableState();
        }
        if(coDriver.startButton.wasActivated()) {
            turret.lockAngle();
        }

        /*
        double testControllerLeftY = -testController.getLeftY();
        if(testControllerLeftY != 0) {
            elevator.setOpenLoop(testControllerLeftY);
            turret.setAngle(0.0);
        } else if(elevator.getState() == Elevator.State.OPEN_LOOP) {
            elevator.lockElevatorHeight();
        }*/
    }

    private void oneControllerMode() {

    }
}
