/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2020.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.SmartTuning;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
* Add your docs here.
*/
public class Turret extends Subsystem {
    SmartTuning smartTuner;
    double smartTunerValue = 0.0;

    LazyTalonFX turret;
    DutyCycle encoder;

    RobotState robotState;
    Swerve swerve;
    
    private Translation2d turretManualVector = new Translation2d();
    public Rotation2d fieldCentricRotation = new Rotation2d();
    private double targetAngle = 0.0;
    private double maxAllowableAngle = Constants.Turret.kMaxControlAngle;
    public void setMaxAllowableAngle(double angle) {
        maxAllowableAngle = angle;
    }
    
    private double stateStartTime = 0.0;
    private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
    
    private boolean zeroedAbsolutely = false;
    public double goalX = 0.0;
    public double goalY = 0.0;

    
    private static Turret instance = null;
    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }
    
    PeriodicIO periodicIO = new PeriodicIO();
    
    public enum ControlState {
        OPEN_LOOP, FIELD_RELATIVE, VISION, POSITION, ROBOT_STATE_VISION, VISION_OFFSET;
    }
    private ControlState currentState = ControlState.OPEN_LOOP;
    public ControlState getState() {
        return currentState;
    }
    
    public List<NetworkTableEntry> targetInfo;

    private boolean visionAngleInRange = false;
    
    private Turret() {
        robotState = RobotState.getInstance();
        swerve = Swerve.getInstance();
        
        turret = new LazyTalonFX(Ports.TURRET, "main");
        encoder = new DutyCycle(new DigitalInput(Ports.TURRET_ENCODER));

        turret.configVoltageCompSaturation(12.0, 10);
        turret.enableVoltageCompensation(true);
        turret.configNominalOutputForward(0.0/12.0, 10);
        turret.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.25));
        
        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turret.setInverted(TalonFXInvertType.Clockwise);
        
        turret.selectProfileSlot(0, 0);
        turret.config_kP(0, Constants.Turret.kP, Constants.kCANTimeoutMs);
        turret.config_kI(0, Constants.Turret.kI, Constants.kCANTimeoutMs);
        turret.config_kD(0, Constants.Turret.kD, Constants.kCANTimeoutMs);
        turret.config_kF(0, Constants.Turret.kF, Constants.kCANTimeoutMs);
        
        //turret.setSelectedSensorPosition(0.0, 0, Constants.kCANTimeoutMs);
        turret.configMotionCruiseVelocity((Constants.Turret.kMaxSpeed * 1.0), Constants.kCANTimeoutMs);
        turret.configMotionAcceleration((Constants.Turret.kMaxSpeed * 2.0), Constants.kCANTimeoutMs); // 3.0
        turret.configMotionSCurveStrength(0); // 0
        turret.configAllowableClosedloopError(0, degreesToEncUnits(0), Constants.kCANTimeoutMs);
        
        turret.configForwardSoftLimitThreshold(degreesToEncUnits(Constants.Turret.kMaxControlAngle), Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitThreshold(degreesToEncUnits(Constants.Turret.kMinControlAngle), Constants.kCANTimeoutMs);
        turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        
        turret.setNeutralMode(NeutralMode.Brake);
        setOpenLoop(0.0);
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        targetInfo = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"),
        table.getEntry("ta"), table.getEntry("tv"));
        
        smartTuner = new SmartTuning(turret, "turret");
        smartTuner.enabled(true);
    }

    public double getAbsoluteEncoderPosition() {
        return encoder.getOutput() * 360.0;
    }
    
    public void setAngle(double angle) {
        if (angle > Constants.Turret.kMaxControlAngle)
            targetAngle = Constants.Turret.kMaxControlAngle;
        else if (angle < Constants.Turret.kMinControlAngle)
            targetAngle = Constants.Turret.kMinControlAngle;
        else
            targetAngle = angle;
        
        periodicIO.demand = degreesToEncUnits(targetAngle);
        if (currentState != ControlState.VISION && currentState != ControlState.ROBOT_STATE_VISION && currentState != ControlState.VISION_OFFSET && currentState != ControlState.FIELD_RELATIVE)
            currentState = ControlState.POSITION;
    }
    
    public void lockAngle() {
        setAngle(getAngle());

        currentState = ControlState.POSITION;
    }
    public void lockTargetAngle() {
        setAngle(targetAngle);
        currentState = ControlState.POSITION;
    }
    
    public void startVision() {
        currentState = ControlState.VISION;
    }

    public void startRobotStateVision() {
        currentState = ControlState.ROBOT_STATE_VISION;
    }
    
    public void startVisionOffset() {
        currentState = ControlState.VISION_OFFSET;
    }
    
    public double getAngle() {
        return encUnitsToDegrees(periodicIO.position);
    }
    /**MagCoder to Falcon's integrated Encoder */
    // public double degreesToEncUnits(double degrees) {
    //     return (int) (degrees / 360 * Constants.Turret.kFalconToEncoderRatio * 2048.0);
    // }
    
    public double encUnitsToDegrees(double encUnits) {
        return encUnits / 2048.0 / Constants.Turret.kFalconToTurretRatio * 360.0;
    }
    
    public int degreesToEncUnits(double degrees) {
        return (int) (degrees / 360.0 * Constants.Turret.kFalconToTurretRatio * 2048.0);
    }
    
    

    public boolean isEncoderConnected() {
        return encoder.getFrequency() != 0.0;
    }
    public void fieldRelativeManual(double xInput, double yInput) {
        currentState = ControlState.FIELD_RELATIVE;
        turretManualVector = new Translation2d(-xInput,yInput);
        Rotation2d swerveRotation = swerve.pose.getRotation();
        Rotation2d manualVectorRotation = turretManualVector.direction().rotateBy(Rotation2d.fromDegrees(-90));
        fieldCentricRotation = swerveRotation.rotateBy(manualVectorRotation.inverse()).inverse();
        double turretFieldRelativeAngle = Util.boundToScope(Constants.Turret.kMaxControlAngle - 360.0, Constants.Turret.kMaxControlAngle, fieldCentricRotation.getDegrees());
        setAngle(turretFieldRelativeAngle);
    }

    public void setOpenLoop(double output) {
        periodicIO.demand = output * 0.5;
        currentState = ControlState.OPEN_LOOP;
    }
    
    public boolean isOpenLoop() {
        return currentState == ControlState.OPEN_LOOP;
    }
    
    public boolean hasReachedAngle() {
        return Math.abs(getAngle() - targetAngle) < Constants.Turret.kAngleTolerance;
    }
    public boolean isReady() {
        return targetInfo.get(3).getDouble(0) == 1.0 && visionAngleInRange && hasReachedAngle();
    }

    public double boundToTurretScope(double turretAngle) {
        turretAngle = Util.placeInAppropriate0To360Scope(getAngle(), turretAngle);
        if (turretAngle > Constants.Turret.kMaxControlAngle)
            turretAngle -= 360.0;
        if (turretAngle < Constants.Turret.kMinControlAngle)
            turretAngle += 360.0;

        return turretAngle;
    }
    
    
    public void resetMotorEncoderPosition(double angle) {
        turret.setSelectedSensorPosition(degreesToEncUnits(angle));
        setOpenLoop(0.0);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = turret.getSelectedSensorPosition();
        periodicIO.velocity = turret.getSelectedSensorVelocity();
        periodicIO.current = turret.getOutputCurrent();
    }
    
    @Override
    public void writePeriodicOutputs() {
        if (currentState == ControlState.OPEN_LOOP) {
            turret.set(ControlMode.PercentOutput, periodicIO.demand);
        } else {
            turret.set(ControlMode.MotionMagic, periodicIO.demand);
        }
    }

    public void updateTurretTuning() {
        smartTuner.update();
        smartTunerValue = smartTuner.getValue();
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Angle", getAngle());
        SmartDashboard.putNumber("Turret Absolute Position", getAbsoluteEncoderPosition());
        SmartDashboard.putBoolean("Turret Is Ready", isReady());

        updateTurretTuning();
        //SmartDashboard.putNumber("Turret Current", periodicIO.current);
        //SmartDashboard.putNumber("Turret Encoder", periodicIO.position);
        //SmartDashboard.putNumber("Turret Velocity", periodicIO.velocity);
        //SmartDashboard.putNumber("Turret Angle Error", targetAngle - getAngle());
        //SmartDashboard.putNumber("Turret Vision Angle", targetInfo.get(0).getDouble(0.0));
    }
    
    private final Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            
        }
        
        @Override
        public void onLoop(double timestamp) {
            if (periodicIO.current > Constants.Turret.kMaxCurrent) {
                DriverStation.reportError("Turret current exceeded max allowed current", false);
            }
            
            switch(currentState) {
                case VISION:
                    Optional<ShooterAimingParameters> params = robotState.getAimingParameters();
                    if (params.isPresent()) {
                        // Aim directly at the vision target
                        double turretAngle = params.get().getTurretToGoal().direction().getDegrees();
                        turretAngle = boundToTurretScope(turretAngle);
                        if (Math.abs(turretAngle - getAngle()) <= Constants.Turret.kAngleTolerance && targetInfo.get(3).getDouble(0) != 1.0) {
                            turretAngle = robotState.getTurretToCenterOfField().direction().getDegrees();
                            turretAngle = boundToTurretScope(turretAngle);
                            
                        }
                        visionAngleInRange = turretAngle >= Constants.Turret.kMinControlAngle && turretAngle <= Constants.Turret.kMaxControlAngle;
                        setAngle(turretAngle);
                    } else {
                        double turretAngle = robotState.getTurretToCenterOfField().direction().getDegrees();
                        turretAngle = boundToTurretScope(turretAngle);
                        visionAngleInRange = turretAngle >= Constants.Turret.kMinControlAngle && turretAngle <= Constants.Turret.kMaxControlAngle;
                        setAngle(turretAngle);
                    }
                    break;
                case ROBOT_STATE_VISION:
                    Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
                    if (aim.isPresent()) {
                        // Compensate for the robot's velocity when aiming
                        double turretAngle = aim.get().getTurretAngle().getDegrees();
                        turretAngle = boundToTurretScope(turretAngle);
                        visionAngleInRange = turretAngle >= Constants.Turret.kMinControlAngle && turretAngle <= Constants.Turret.kMaxControlAngle;
                        double offsetAngle = 0;
                        /*if (turretAngle < Constants.Turret.kMinControlAngle || turretAngle > Constants.Turret.kMaxControlAngle) {
                            if (swerve.getVelocity().norm() < 0.01) {
                                double offsetAngle = 0.0;
                                if (Math.abs(getAngle() - Constants.Turret.kMaxControlAngle) < Math.abs(getAngle() - Constants.Turret.kMinControlAngle)) {
                                    offsetAngle = Math.toDegrees(Rotation2d.fromDegrees(turretAngle).distance(Rotation2d.fromDegrees(Constants.Turret.kMaxControlAngle - 5.0)));
                                } else {
                                    offsetAngle = Math.toDegrees(Rotation2d.fromDegrees(turretAngle).distance(Rotation2d.fromDegrees(Constants.Turret.kMinControlAngle + 5.0)));
                                }
                                swerve.rotate(swerve.getPose().getRotation().getUnboundedDegrees() + offsetAngle);
                                System.out.println("Turret vision angle = " + turretAngle + "; offsetAngle = " + offsetAngle);
                            }
                        }*/
         
                        setAngle(turretAngle);
                    }

                    break;
                default:
                break;
            }
        }
        
        @Override
        public void onStop(double timestamp) {
            
        }
    };
    
    public Request angleRequest(double angle, double speedScalar, boolean wait) {
        return new Request(){
            
            @Override
            public void act() {
                turret.configMotionCruiseVelocity((int)(Constants.Turret.kMaxSpeed * speedScalar), Constants.kCANTimeoutMs);
                setAngle(angle);
            }
            
            @Override
            public boolean isFinished() {
                if (wait) {
                    return hasReachedAngle();
                } else {
                    return true;
                }
            }
            
        };
    }
    
    public Request angleRequest(double angle) {
        return angleRequest(angle, 1.0, true);
    }
    
    public Request angleRequest(double angle, double speedScalar) {
        return angleRequest(angle, speedScalar, true);
    }
    
    public Request lockAngleRequest() {
        return new Request(){
            
            @Override
            public void act() {
                lockAngle();
            }
        };
    }
    public Request resetTurretPositionRequest() {
        return new Request() {
            
            @Override
            public void act() {
                setAngle(0.0);
            }
        };
    }
    
    public Request openLoopRequest(double output) {
        return new Request(){
            
            @Override
            public void act() {
                setOpenLoop(output);
            }
        };
    }
    
    public Request startVisionRequest() {
        return new Request(){
            
            @Override
            public void act() {
                startVision();
            }
            
            @Override
            public boolean isFinished() {
                return hasReachedAngle();
            }
            
        };
    }

    public Request robotStateVisionRequest() {
        return new Request() {
            
            @Override
            public void act() {
                startRobotStateVision();
            }

            @Override
            public boolean isFinished() {
                return targetInfo.get(3).getDouble(0) == 1.0 && visionAngleInRange && hasReachedAngle();
            }
        };
    }

    
    
    
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }
    public void zeroTurret() {
        zeroedAbsolutely = true;
    }
    public void resetToAbsolutePosition() {
        if(!zeroedAbsolutely) {
            double cancoderOffset = Util.boundAngle0to360Degrees(getAbsoluteEncoderPosition() - Constants.Turret.kTurretStartingEncoderPosition);
            double absoluteTurretAngle = Constants.Turret.kTurretStartingAngle + (cancoderOffset / Constants.Turret.kEncoderToTurretRatio);
            if (absoluteTurretAngle > Constants.Turret.kMaxInitialAngle) {
                cancoderOffset -= 360.0;
                absoluteTurretAngle = Constants.Turret.kTurretStartingAngle + (cancoderOffset / Constants.Turret.kEncoderToTurretRatio);
            } else if (absoluteTurretAngle < Constants.Turret.kMinInitialAngle) {
                cancoderOffset += 360.0;
                absoluteTurretAngle = Constants.Turret.kTurretStartingAngle + (cancoderOffset / Constants.Turret.kEncoderToTurretRatio);
            }

            if (absoluteTurretAngle > Constants.Turret.kMaxInitialAngle || absoluteTurretAngle < Constants.Turret.kMinInitialAngle) {
                DriverStation.reportError("Turret angle is out of bounds", false);
            }

            turret.setSelectedSensorPosition(degreesToEncUnits(absoluteTurretAngle), 0, Constants.kCANTimeoutMs);
        }
    }
    
    @Override
    public void stop() {
        setOpenLoop(0.0);        
    }
    
    private static class PeriodicIO {
        //Inputs
        public double position;
        public double velocity;
        public double voltage;
        public double current;
        
        //Outputs
        public double demand;
    }
    
}