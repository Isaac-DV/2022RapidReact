// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;


import java.util.Optional;
import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.SmartTuner;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Column extends Subsystem {
    SmartTuner smartTuner;
    
    Shooter shooter;
    Turret turret;
    MotorizedHood motorizedHood;
    BallFeeder ballFeeder;


    LazyTalonFX column;
    DigitalInput banner;

    private boolean notifyDrivers = false;

    boolean detectedBall = false;
    boolean previousDetectedBall = false;

    boolean pendingShutdown = false;
    boolean previousFeederDetected = false;

    boolean shootingCurrentBall = false;
    boolean previousShootingCurrentBall = false;
    public boolean isShootingCurrentBall() {
        return shootingCurrentBall;
    }


    double columnStartTimestamp = Double.POSITIVE_INFINITY;
    double ballDetectedTimestamp = 0.0;
    double ballLeftTimestamp = 0.0;
    double disabledSwerveTimestamp = 0.0;
    double targetRPM = 0.0;
    int fastFedBalls = 0;
    boolean isFastFeeding = false;

    private int loadedBallCount = 0;
    public int getLoadedBallCount() {
        return loadedBallCount;
    }

    private int totalBallCount = 0;
    public int getTotalBallCount() {
        return totalBallCount;
    }

    private static Column instance = null;
    public static Column getInstance() {
        if (instance == null)
            instance = new Column();
        return instance;
    }
    public Column() {
        shooter = Shooter.getInstance();
        turret = Turret.getInstance();
        motorizedHood = MotorizedHood.getInstance();
        ballFeeder = BallFeeder.getInstance();

        column = new LazyTalonFX(Ports.COLUMN, "main");
        banner = new DigitalInput(Ports.COLUMN_BANNER);

        column.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        column.enableVoltageCompensation(true);
        column.setInverted(TalonFXInvertType.CounterClockwise);
        column.setNeutralMode(NeutralMode.Brake);
        column.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
        column.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);

        column.config_kP(0, 0.0, Constants.kCANTimeoutMs); //0.2 : 0.1
        column.config_kI(0, 0.0, Constants.kCANTimeoutMs); //0.0 : 0.0
        column.config_kD(0, 0.0, Constants.kCANTimeoutMs); //4.0 : 1.0
        column.config_kF(0, 0.051, Constants.kCANTimeoutMs); //0.051
        column.selectProfileSlot(0, 0);
        column.config_IntegralZone(0, getRPMToEncVelocity(100));

        AsynchronousInterrupt interrupt = new AsynchronousInterrupt(banner, new BiConsumer<Boolean,Boolean>() {

            @Override
            public void accept(Boolean risingEdge, Boolean fallingEdge) {
                if(getState() == ControlState.INDEX_BALLS) {
                    if(getBanner()) {
                        //column.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
                        setOpenLoop(0.0);
                    }
                }
                if(getBanner()) {
                    ballDetectedTimestamp = Timer.getFPGATimestamp();
                    detectedBall = true;
                    System.out.println("Column banner detected ball");
                } else {
                    ballDetectedTimestamp = Double.POSITIVE_INFINITY;
                    detectedBall = false;
                    if (shootingCurrentBall) {
                        ballLeftTimestamp = Timer.getFPGATimestamp();
                    }
                    shootingCurrentBall = false;
                    System.out.println("Column banner stopped detecting ball");
                }
            }        
        });
        interrupt.setInterruptEdges(true, true);
        interrupt.enable();

        smartTuner = new SmartTuner(column, "column");
        smartTuner.enabled(false);
    }
    public boolean getBanner() {
        return banner.get();
    }

    public enum ControlState {
        OFF(0.0), FEED_BALLS(0.0), MANUAL_FEED_BALLS(Constants.Column.kFeedBallSpeed), EJECT(Constants.Column.kReverseSpeed), 
        INDEX_BALLS(0.0), INTAKE(Constants.Column.kFeedBallSpeed), VELOCITY(0.0);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {
        return currentState;
    }
    public void setState(ControlState desiredState) {
        currentState = desiredState;
    }
    public void setOpenLoop(double demand) {
        column.set(ControlMode.PercentOutput, demand);
    }
    public void setVelocityState(double rpm) {
        setState(ControlState.VELOCITY);
        setVelocity(rpm);
    }
    public void setVelocity(double rpm) {
        targetRPM = rpm;
        column.set(ControlMode.Velocity, getRPMToEncVelocity(rpm));
    }
    public double getRPMToEncVelocity(double rpm) {
        return rpm * 2048.0 / 600.0 * 1.0;
    }
    public synchronized double encVelocityToRPM(double encVelocity) {
        return encVelocity / 2048.0 * 600.0 / 1.0;
    }

    public void conformToState(ControlState desiredState, double demand) {
        setState(desiredState);
        setOpenLoop(demand);
    }
    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }

    private boolean allSubsystemsReady() {
        return shooter.hasReachedSetpoint() && turret.isReady() && motorizedHood.hasReachedAngle();
    }

    public void shutDownIfUnused() {
        if (getState() == ControlState.INDEX_BALLS && !ballFeeder.hasSentUpBall()) {
            conformToState(ControlState.OFF);
        }
    }

    private double getFeedingDelay() {
        Optional<ShooterAimingParameters> aim = RobotState.getInstance().getCachedAimingParameters();
        if (aim.isPresent()) {
            if (aim.get().getRange() > Constants.Column.kMaxDistance) {
                return Constants.Column.kMinFeedDelay;
            }
            if (aim.get().getRange() < Constants.Column.kMinDistance) {
                return Constants.Column.kMaxFeedDelay;
            }
            double scaledDelay = Constants.Column.kMinFeedDelay + 
                    ((Constants.Column.kMaxFeedDelay - Constants.Column.kMinFeedDelay) * 
                            (Constants.Column.kMaxDistance - aim.get().getRange()) / (Constants.Column.kMaxDistance - Constants.Column.kMinDistance));
            
            return scaledDelay;
        }

        return Constants.Column.kMaxFeedDelay;
    }

    private void printVisionSubsystemInfo() {
        Optional<ShooterAimingParameters> aim = RobotState.getInstance().getCachedAimingParameters();
        if (aim.isPresent()) {
            System.out.println("SHOOTING BALL:");
            System.out.println("Distance to goal: " + aim.get().getRange() + ", robot velocity: " + Swerve.getInstance().getVelocity().toString());
            System.out.println("Turret -> current angle: " + turret.getAngle() + ", vision angle: " + aim.get().getTurretAngle().getDegrees());
            System.out.println("Shooter -> current RPM: " + shooter.getLeftRPM() + ", vision RPM: " + aim.get().getShooterRPM());
            System.out.println("Hood -> current angle: " + motorizedHood.getAngle() + ", vision angle: " + aim.get().getHoodAngle().getDegrees());
            System.out.println("Match Time: " + DriverStation.getMatchTime());
        } else {
            System.out.println("SHOOTING BALL BUT NO TARGET PRESENT");
        }
    }

    private boolean inRange(Optional<ShooterAimingParameters> aim, double floor, double ceiling) {
        if (aim.isPresent()) {
            return floor <= aim.get().getRange() && aim.get().getRange() <= ceiling &&
                    Swerve.getInstance().getVelocity().norm() <= Constants.Column.kFasterFeedMaxSwerveVelocity && 
                    !(Intake.getInstance().getState() == Intake.ControlState.INTAKE || Intake.getInstance().getState() == Intake.ControlState.AUTO_FEED_INTAKE) &&
                    fastFedBalls < 1;
        }

        return false;
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            column.setNeutralMode(NeutralMode.Brake);

        }

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                case FEED_BALLS:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getCachedAimingParameters();
                    if (!getBanner()) {
                        if (timestamp - ballLeftTimestamp <= Constants.Column.kFasterFeedDuration && 
                                inRange(aim, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)) {
                            setVelocity(Constants.Column.kFasterFeedVelocity);
                        } else {
                            setVelocity(Constants.Column.kQueueVelocitySpeed);
                        }
                    } else if((getBanner() && Double.isFinite(ballDetectedTimestamp) && 
                            (timestamp - ballDetectedTimestamp) >= Constants.Column.kBallDelay && allSubsystemsReady())) {
                        //setOpenLoop(Constants.Column.kFeedBallSpeed);
                        columnStartTimestamp = timestamp;
                        if (timestamp - ballLeftTimestamp <= Constants.Column.kFasterFeedDuration &&
                                inRange(aim, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)) {
                            setVelocity(Constants.Column.kFasterFeedVelocity);
                            if (!shootingCurrentBall) {
                                fastFedBalls++;
                            }
                        } else {
                            setVelocity(Constants.Column.kFeedVelocitySpeed);
                            if (!shootingCurrentBall) {
                                fastFedBalls = 0;
                            }
                        }
                        if (!shootingCurrentBall) {
                            printVisionSubsystemInfo();
                            Swerve.getInstance().enableInputs(false);
                            disabledSwerveTimestamp = timestamp;
                        }
                        shootingCurrentBall = true;
                        //System.out.println("Shot the ball at a range of : " + shooter.getTargetRange());
                    } else {
                        if (!shootingCurrentBall && 
                                (timestamp - ballLeftTimestamp >= Constants.Column.kFasterFeedDuration || !inRange(aim, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)))
                            setOpenLoop(0.0);
                    }
                    break;
                case MANUAL_FEED_BALLS:
                    System.out.println("Column Banner : " + getBanner() + ", Ball Detected Timestamp finite : " + Double.isFinite(ballDetectedTimestamp) + ", within timestamp : " + ((timestamp - ballDetectedTimestamp) >= Constants.Column.kBallDelay) + ", Shooter on Target : " + shooter.hasReachedSetpoint() + ", Hood on Target : " + motorizedHood.hasReachedAngle());
                    Optional<ShooterAimingParameters> aimManual = RobotState.getInstance().getCachedAimingParameters();
                    if (!getBanner()) {
                        if (timestamp - ballLeftTimestamp <= Constants.Column.kFasterFeedDuration && 
                                inRange(aimManual, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)) {
                            setVelocity(Constants.Column.kFasterFeedVelocity);
                        } else {
                            setVelocity(Constants.Column.kQueueVelocitySpeed);
                        }
                    } else if(getBanner() && Double.isFinite(ballDetectedTimestamp) && (timestamp - ballDetectedTimestamp) >= Constants.Column.kBallDelay
                                && shooter.hasReachedSetpoint() && motorizedHood.hasReachedAngle()) {
                        columnStartTimestamp = timestamp;
                        shootingCurrentBall = true;
                        if (timestamp - ballLeftTimestamp <= Constants.Column.kFasterFeedDuration &&
                                inRange(aimManual, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)) {
                            setVelocity(Constants.Column.kFasterFeedVelocity);
                        } else {
                            setVelocity(Constants.Column.kFeedVelocitySpeed);
                        }
                    } else {
                        if (!shootingCurrentBall && 
                                (timestamp - ballLeftTimestamp >= Constants.Column.kFasterFeedDuration || !inRange(aimManual, Constants.Column.kMinFasterFeedRange, Constants.Column.kMaxFasterFeedRange)))
                            setOpenLoop(0.0);
                    }
                    break;
                case INDEX_BALLS:
                    if(!getBanner() && !detectedBall) {
                        //column.configOpenloopRamp(0.1, Constants.kCANTimeoutMs);
                        setVelocity(Constants.Column.kQueueVelocitySpeed);
                    } else if(getBanner() && detectedBall) {
                        //column.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
                        setOpenLoop(0.0);
                    }
                    
                    if(getBanner() != detectedBall) {
                        detectedBall = !detectedBall;
                    }

                    break;
                default:
                break;
            }
            
            if(ballFeeder.hasSentUpBall() && getBanner()) {
                ballFeeder.setSentUpBall(false);
            }
            if(banner.get() && ballFeeder.detectedBallType == BallFeeder.BallType.Team) {
                notifyDrivers = true;
            }

            if (timestamp - disabledSwerveTimestamp >= Constants.Column.kColumnActivationShotTime) {
                Swerve.getInstance().enableInputs(true);
            }

            if (!shootingCurrentBall && timestamp - ballLeftTimestamp >= Constants.Column.kExtraSwerveDelay) {
                Swerve.getInstance().enableInputs(true);

            }
            
        }

        @Override
        public void onStop(double timestamp) {
            
        }
        
    };
    public boolean needsToNotifyDrivers() {
        if(notifyDrivers) {
            notifyDrivers = false;
            return true;
        }
        return false;
    }

   

    public Request stateRequest(ControlState desiredState) {
        return new Request() {

            @Override
            public void act() {
                conformToState(desiredState);
            }  
        };
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Column Banner Sensor", getBanner());
        SmartDashboard.putString("Column State", getState().toString());
        if(Settings.debugColumn()) {
            SmartDashboard.putNumber("Column banner detected timestamp", (Timer.getFPGATimestamp() - ballDetectedTimestamp));
            SmartDashboard.putNumber("Column RPM", encVelocityToRPM(column.getSelectedSensorVelocity()));
            SmartDashboard.putNumber("Column RPM Target", targetRPM);
            SmartDashboard.putNumber("Column Total Ball Counter", totalBallCount);
            SmartDashboard.putNumber("Column Loaded Ball Counter", loadedBallCount);
            smartTuner.update();
        }
        /*if(column.getBusVoltage() == 0)
            DriverStation.reportError("COLUMN MOTOR NOT DETECTED", false);*/
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {
        conformToState(ControlState.OFF);
    }

}
