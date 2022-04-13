/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2020;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.BallSplitter;
import com.team1323.frc2020.subsystems.Column;
import com.team1323.frc2020.subsystems.DoubleTelescopes;
import com.team1323.frc2020.subsystems.Intake;
import com.team1323.frc2020.subsystems.LEDs;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.Shooter;
import com.team1323.frc2020.subsystems.SubsystemManager;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team1323.frc2020.subsystems.Swerve;
import com.team1323.frc2020.subsystems.Turret;
import com.team1323.frc2020.subsystems.Wrist;
import com.team1323.io.Xbox;
import com.team254.lib.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	Xbox driver, coDriver, singleController, testController;

    private Swerve swerve;
    private Intake intake;
    private Wrist wrist;
    private BallSplitter ballSplitter;
    private BallFeeder ballFeeder;
    private Column column;
    private Turret turret;
    private MotorizedHood motorizedHood;
    private Shooter shooter;
    private DoubleTelescopes doubleTelescopes;
    private LEDs leds;
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
        testController = new Xbox(4);
        singleController = new Xbox(5);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.25);
		coDriver.rightBumper.setLongPressDuration(1.0);

        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        wrist = Wrist.getInstance();
        ballSplitter = BallSplitter.getInstance();
        ballFeeder = BallFeeder.getInstance();
        column = Column.getInstance();
        turret = Turret.getInstance();
        motorizedHood = MotorizedHood.getInstance();
        shooter = Shooter.getInstance();
        doubleTelescopes = DoubleTelescopes.getInstance();
        leds = LEDs.getInstance();

        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(
				Arrays.asList(swerve, intake, wrist, ballSplitter, ballFeeder, turret, doubleTelescopes,
                    motorizedHood, shooter, column, leds, s));
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
            motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle);
        }
        swerve.setDriveNeutralMode(NeutralMode.Brake);
        wrist.setWristLocked();
        leds.configLEDs(LEDs.LEDColors.OFF);
    }

    @Override
    public void onLoop(double timestamp) {
        //RobotState.getInstance().outputToSmartDashboard();
        if(inAuto) {
            // Any auto-specific LED controls can go here
        } else {
            driver.update();
			coDriver.update();
            //singleController.update();
            //testController.update();
            if(oneControllerMode)
                singleController.update();
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
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, driver.leftBumper.isBeingPressed() , false);
        
        if (driver.bButton.wasActivated())
            swerve.rotate(90);
        else if (driver.aButton.wasActivated()) 
            swerve.rotate(180);
        else if (driver.xButton.wasActivated())
            //swerve.rotate(270);
            turret.setCOFState();
        else if (driver.yButton.wasActivated())
            swerve.rotate(0.0);

        if (driver.startButton.isBeingPressed()) 
            swerve.setState(Swerve.ControlState.NEUTRAL);

        if (driver.backButton.wasActivated()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
        }
        /*if(driver.rightBumper.isBeingPressed())
            swerve.rotate(ballSplitter.getEjectRotation(BallSplitter.EjectLocations.TEAM_TERMINAL.location));
        else if(driver.leftBumper.isBeingPressed())
            swerve.rotate(ballSplitter.getEjectRotation(BallSplitter.EjectLocations.TEAM_HANGER.location));*/

        if(s.needsToNotifyDrivers()) {
            driver.rumble(1.0, 2.0);
            coDriver.rumble(1.0, 2.0);
        }
        if(driver.leftTrigger.wasActivated()) {
            swerve.setMaxSpeed(0.5);
            //motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle);
        } else if(driver.leftTrigger.wasReleased()) {
            swerve.setMaxSpeed(1.0);
        }
        /*if(driver.rightTrigger.wasActivated()) {
            swerve.useSlewLimiter(true);
            swerve.setMaxSpeed(0.5);
        } else if(driver.rightTrigger.wasReleased()) {
            swerve.useSlewLimiter(false);
            swerve.setMaxSpeed(1.0);
        }*/
        if(driver.POV180.wasActivated()) {
            motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle);
        }
        if(driver.POV0.wasActivated()) {
            swerve.resetPosition(Constants.kLaunchPadPose);
        }
        if(driver.rightBumper.wasActivated()) {
            motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle);
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

        /*if(coDriver.POV90.wasActivated()) {
            motorizedHood.setAngle(45);
            System.out.println("POV90 Ran");
        } 
        if(coDriver.POV180.wasActivated()) {
            motorizedHood.setAngle(15);
        }*/
        /*
        if(coDriverLeftY != 0) {
            wrist.setOpenLoop(coDriverLeftY);
        } else if(wrist.getState() == Wrist.State.OPEN_LOOP) {
            wrist.setWristLockedAtCurrentAngle();
        }*/
        if(coDriverRightX != 0) {
            turret.setOpenLoop(coDriverRightX);
        } else if (!doubleTelescopes.bothTelescopesZeroed()) {
            turret.setOpenLoop(0.0);
        } else if(turret.getState() == Turret.ControlState.OPEN_LOOP) {
            turret.lockAngle();
        }
        

        
        if(!doubleTelescopes.liftModeEnabled()) {
            if(driver.rightCenterClick.wasActivated()) {
                turret.setAngle(0.0);
            }

            if(coDriver.rightCenterClick.wasActivated()) {
                turret.setAngle(0.0);
            }

            if(coDriver.leftCenterClick.wasActivated()) {
                turret.setCOFState();
            }

            if(coDriver.aButton.isBeingPressed()) {
                if(column.getState() != Column.ControlState.FEED_BALLS) {
                    column.setState(Column.ControlState.INDEX_BALLS);
                }
            }

            if(coDriver.rightBumper.wasActivated()) {
                intake.conformToState(Intake.ControlState.INTAKE);
                ballFeeder.setState(BallFeeder.State.DETECT);
                if(column.getState() != Column.ControlState.FEED_BALLS) {
                    column.setState(Column.ControlState.INDEX_BALLS);
                }
                ballFeeder.setPrintFeeder(true);
            } else if(coDriver.rightBumper.wasReleased()) {
                intake.conformToState(Intake.ControlState.OFF);
                ballFeeder.queueShutdown(true);
                column.shutDownIfUnused();
                ballFeeder.setPrintFeeder(false);
            }

            if(coDriver.aButton.wasActivated()) {
                wrist.setWeakIntakeState(true);
                intake.conformToState(Intake.ControlState.INTAKE);
                wrist.setWristAngleWithAcceleration(Constants.Wrist.kIntakeAngle);
                ballFeeder.setState(BallFeeder.State.DETECT);
                if(column.getState() != Column.ControlState.FEED_BALLS) {
                    column.setState(Column.ControlState.INDEX_BALLS);
                }
                ballFeeder.setPrintFeeder(true);
            } else if(coDriver.aButton.wasReleased()) {
                wrist.setWeakIntakeState(false);
                wrist.setLowStatorLimit(false);
                intake.conformToState(Intake.ControlState.OFF);
                ballFeeder.queueShutdown(true);
                wrist.setWristAngleWithAcceleration(Constants.Wrist.kBallDebouncerAngle);
                column.shutDownIfUnused();
                ballFeeder.setPrintFeeder(false);
            }

            if(coDriver.bButton.wasActivated()) {
                wrist.setWristAngleWithAcceleration(Constants.Wrist.kStowedAngle);
            }

            if(coDriver.yButton.wasActivated()) {
                s.positionShotState(); //2250, 19
            } else if(coDriver.yButton.wasReleased()) {
                s.postShotState();
            }
            /*if(coDriver.xButton.wasActivated()) {
                s.manualShotState(1900, 14);
            } else if(coDriver.xButton.wasReleased()) {
                s.postShotState();
            }*/
            if(coDriver.xButton.wasActivated()) {
                s.manualShotState(shooter.dashboardRPMInput, motorizedHood.angleInput);
                turret.startVision();
            } else if(coDriver.xButton.wasReleased()) {
                s.postShotState();
            }
            if(coDriver.rightTrigger.wasActivated() || driver.rightTrigger.wasActivated()) {
                SmartDashboard.putBoolean("Vision Shot is activated", true);
                if(driver.rightTrigger.isBeingPressed())
                    swerve.setMaxSpeed(0.5);
                s.visionShotState();
            } else if((coDriver.rightTrigger.wasReleased() && !driver.rightTrigger.isBeingPressed()) ||
                    (driver.rightTrigger.wasReleased() && !coDriver.rightTrigger.isBeingPressed())) {
                SmartDashboard.putBoolean("Vision Shot is activated", false);
                swerve.setMaxSpeed(1.0);
                s.postShotState();
            }

            if(coDriver.startButton.wasActivated()) {
                //turret.lockAngle();
                turret.startVision();
                motorizedHood.setState(MotorizedHood.State.VISION);
            }

        } else {
            if(coDriver.aButton.wasActivated()) {
                doubleTelescopes.setLiftMode(DoubleTelescopes.LiftMode.FIRST_WINCH);
            }
            if(coDriver.bButton.wasActivated()) {
                if(doubleTelescopes.leftTelescopeOnTarget() && doubleTelescopes.rightTelescopeOnTarget())
                    doubleTelescopes.setLiftMode(DoubleTelescopes.LiftMode.THIRD_INITIAL_HANG);
            }
            if(coDriver.yButton.wasActivated()) {
                doubleTelescopes.setLiftMode(DoubleTelescopes.LiftMode.SECOND_FULL_RELEASE);
            }
        }

        if(coDriver.POV180.wasActivated()) {
            doubleTelescopes.disableLiftMode();
        }
        if(coDriver.POV0.wasActivated()) {
            s.startHangSequence();
        }

        if (coDriver.leftTrigger.wasActivated()) {
            motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle);
        }
        
       
        

        
        if(coDriver.leftBumper.wasActivated()) {
            s.reverseAllSubsystems();
        } else if(coDriver.leftBumper.wasReleased()) {
            s.disableState();
            ballFeeder.setState(BallFeeder.State.DETECT);
        }

        if(column.needsToNotifyDrivers()) {
            coDriver.rumble(2.0, 1.0);
            driver.rumble(2.0, 1.0);
        }
        if(coDriver.backButton.wasActivated()) {
            s.disableState();
        }
        if(singleController.backButton.wasActivated()) {
            s.disableState();
        }
        
        
    }

    private void oneControllerMode() {
        double swerveYInput = singleController.getLeftX();
        double swerveXInput = -singleController.getLeftY();
        double swerveRotationInput = singleController.getRightX();
        
        swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, singleController.leftTrigger.isBeingPressed());
        
        if (singleController.startButton.isBeingPressed()) 
            swerve.setState(Swerve.ControlState.NEUTRAL);

        if (singleController.backButton.shortReleased() || singleController.backButton.longPressed()) {
            swerve.temporarilyDisableHeadingController();
            swerve.zeroSensors(new Pose2d());
            swerve.resetAveragedDirection();
            s.disableState();
        }
        
        
        /*if(singleController.xButton.wasActivated()) {
            motorizedHood.setAngleState(Constants.MotorizedHood.kMinControlAngle + motorizedHood.angleInput); //25.0
            shooter.setVelocity(shooter.dashboardRPMInput); //2100
            turret.startVision();
            column.setState(Column.ControlState.FEED_BALLS);
            turret.startVision();
        } else if(singleController.xButton.wasReleased()) {
            s.postShotState();
        }

        if(singleController.aButton.isBeingPressed()) {
            if(column.getState() != Column.ControlState.FEED_BALLS) {
                column.setState(Column.ControlState.INDEX_BALLS);
            }
        }
        
        if(singleController.aButton.wasActivated()) {
            intake.conformToState(Intake.ControlState.INTAKE);
            wrist.setWristAngle(Constants.Wrist.kIntakeAngle);
            ballFeeder.setState(BallFeeder.State.DETECT);
            if(column.getState() != Column.ControlState.FEED_BALLS) {
                column.setState(Column.ControlState.INDEX_BALLS);
            }
        } else if(singleController.aButton.wasReleased()) {
            intake.conformToState(Intake.ControlState.OFF);
            ballFeeder.queueShutdown(true);
            wrist.setWristAngle(Constants.Wrist.kLowestAngle);
        }*/

        if (singleController.leftTrigger.wasActivated()) {
            if (singleController.aButton.isBeingPressed()) {
                wrist.setWristAngle(Constants.Wrist.kBallDebouncerAngle);
            }
        } else if (singleController.leftTrigger.wasReleased()) {
            if (singleController.aButton.isBeingPressed()) {
                wrist.setWristAngle(Constants.Wrist.kIntakeAngle);
            }
        }
        /*if(singleController.bButton.wasActivated()) {
            wrist.setWristAngle(Constants.Wrist.kStowedAngle);
        }*/
        if(column.needsToNotifyDrivers()) {
            singleController.rumble(2.0, 1.0);
        }
        if(singleController.startButton.wasActivated()) {
            //turret.lockAngle();
            turret.startVision();
        }


        if(singleController.leftBumper.wasActivated()) {
            s.reverseAllSubsystems();
        } else if(singleController.leftBumper.wasReleased()) {
            s.disableState();
        }
           

        if(singleController.rightTrigger.wasActivated()) {
            s.visionShotState();
        } else if(singleController.rightTrigger.wasReleased()) {
            s.postShotState();
        }
        

        if(singleController.rightCenterClick.wasActivated()) {
            turret.setAngle(0.0);
        }
        
    }
}
