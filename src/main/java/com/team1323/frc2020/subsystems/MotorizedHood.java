
package com.team1323.frc2020.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.SmartTuner;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An adjustable shooter hood, powered by a BAG motor
 */
public class MotorizedHood extends Subsystem {
    SmartTuner smartTuner;

    private static MotorizedHood instance = null;
    public static MotorizedHood getInstance() {
        if (instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    LazyTalonFX hood;
    RobotState robotState;
    
    public double angleInput = 0;
    private boolean isEncoderFlipped = false;
    private boolean zeroedAbsolutely = false;
    private boolean isFirstEnable = true;
    public enum State {
        ZEROING, POSITION, VISION, ROBOT_POSITION
    }
    private State currentState = State.POSITION;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        if (currentState != State.ZEROING) {
            currentState = desiredState;
        }
    }
    public PeriodicIO periodicIO = new PeriodicIO();

    public MotorizedHood() {
        hood = new LazyTalonFX(Ports.HOOD_TALON, "main");

        hood.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        hood.enableVoltageCompensation(true);
        hood.setInverted(TalonFXInvertType.Clockwise);
        setEncoderPhase(false);

        hood.setNeutralMode(NeutralMode.Brake);

        hood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        setCurrentLimit(10.0);
 
        hood.selectProfileSlot(0, 0);
        hood.config_kP(0, Constants.MotorizedHood.kP, Constants.kCANTimeoutMs);
        hood.config_kI(0, Constants.MotorizedHood.kI, Constants.kCANTimeoutMs);
        hood.config_kD(0, Constants.MotorizedHood.kD, Constants.kCANTimeoutMs);
        hood.config_kF(0, Constants.MotorizedHood.kF, Constants.kCANTimeoutMs);

        hood.configMotionCruiseVelocity(Constants.MotorizedHood.kMaxSpeed * 1.0, Constants.kCANTimeoutMs);
        hood.configMotionAcceleration(Constants.MotorizedHood.kMaxSpeed * 10.0, Constants.kCANTimeoutMs);

        hood.configForwardSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorizedHood.kMaxControlAngle), Constants.kCANTimeoutMs);
        hood.configReverseSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorizedHood.kMinControlAngle), Constants.kCANTimeoutMs);
        enableLimits(true);

        hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        hood.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);
        
        hood.setSelectedSensorPosition(degreesToEncUnits(15.0));
        //hood.setSelectedSensorPosition(0);

        setOpenLoop(0.0);
        smartTuner = new SmartTuner(hood, "hood");
        smartTuner.enabled(true);

        initializeDashboardValues();
    }

    private void enableLimits(boolean enable) {
        hood.configForwardSoftLimitEnable(enable, Constants.kCANTimeoutMs);
        hood.configReverseSoftLimitEnable(enable, Constants.kCANTimeoutMs);
    }

    private void setCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 10);
        hood.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs);
    }

    private void setEncoderPhase(boolean phase) {
        isEncoderFlipped = phase;
    }

    private double getAbsoluteEncoderDegrees() {
        return 0;
        //return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
    }

    private boolean isEncoderConnected() {
        /*if (RobotBase.isReal()) {
            return encoder.getFrequency() != 0;
        }*/
        return false;
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / Constants.MotorizedHood.kTicksPerDegree;
    }

    public double degreesToEncUnits(double degrees) {
        return (degrees * Constants.MotorizedHood.kTicksPerDegree);
    }

    public double encUnitsToHoodAngle(double encUnits) {
        return encUnitsToDegrees(encUnits);
    }

    public double hoodAngleToEncUnits(double degrees) {
        return (degreesToEncUnits(degrees));
    }

    public static double physicalAngleToEmpiricalAngle(double physicalAngle) {
        double physicalAngleOffset = physicalAngle - Constants.MotorizedHood.kMinControlAngle;

        return Constants.MotorizedHood.kMaxEmpiricalAngle - physicalAngleOffset;
    }

    public static double empiricalAngleToPhysicalAngle(double empiricalAngle) {
        double empiricalAngleOffset = Constants.MotorizedHood.kMaxEmpiricalAngle - empiricalAngle;

        return Constants.MotorizedHood.kMinControlAngle + empiricalAngleOffset;
    }

    public double getAngle() {
        return encUnitsToHoodAngle(periodicIO.position);
    }

    public void setAngle(double angle) {
        if (currentState != State.ZEROING) {
            angle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);

            periodicIO.controlMode = ControlMode.MotionMagic;
            periodicIO.demand = hoodAngleToEncUnits(angle);
        }
    }
    public void setAngleState(double angle) {
        if (currentState != State.ZEROING) {
            setState(State.POSITION);
            setAngle(angle);
        }
    }
    public void lockAngle() {
        if (currentState != State.ZEROING) {
            setState(State.POSITION);
            periodicIO.controlMode = ControlMode.MotionMagic;
            periodicIO.demand = periodicIO.position;
        }
    }

    public boolean hasReachedAngle() {
        return periodicIO.controlMode == ControlMode.MotionMagic && 
            Math.abs(encUnitsToDegrees(periodicIO.demand - periodicIO.position)) <= Constants.MotorizedHood.kAngleTolerance;
    }

    public void setOpenLoop(double percentOutput) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = percentOutput;
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            hood.setNeutralMode(NeutralMode.Brake);
            if (isFirstEnable) {
                zeroWithHardStop();
                isFirstEnable = false;
            } 
        }

        @Override
        public void onLoop(double timestamp) {
            smartTuner.update();   
            angleInput = smartTuner.getValue();
            switch (currentState) {
                case VISION:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getCachedAimingParameters();
                    if (aim.isPresent()) {
                        setAngle(aim.get().getHoodAngle().getDegrees());
                    }
                    break;
                case ROBOT_POSITION:
                    Optional<ShooterAimingParameters> positionParameters = RobotState.getInstance().getAimingParametersFromPosition();
                    if(positionParameters.isPresent()) {
                        setAngle(positionParameters.get().getHoodAngle().getDegrees());
                    }
                    /*Translation2d robotToCenterVector = RobotState.getInstance().getTurretToCenterOfField();
                    double robotToCenterMagnitude = robotToCenterVector.norm();
                    Translation2d shotVector = Constants.kDistanceToShotVectorMap.getInterpolated(new InterpolatingDouble(robotToCenterMagnitude));
                    setAngle(shotVector.direction().getDegrees());*/
                    break;
                case ZEROING:
                    if (hood.getOutputCurrent() > Constants.MotorizedHood.kZeroingCurrent) {
                        stop();
                        hood.setSelectedSensorPosition(degreesToEncUnits(Constants.MotorizedHood.kHoodStartingAngle), 0, 0);
                        System.out.println("Zeroed el hood");
                        enableLimits(true);
                        currentState = State.VISION;
                    }
                    break;
                default:
                    break;
            }       
        }

        @Override
        public void onStop(double timestamp) {
            hood.setNeutralMode(NeutralMode.Brake);
        }
        
    };


    

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = hood.getSelectedSensorPosition(0);
    }

    @Override
    public void writePeriodicOutputs() {
        hood.set(periodicIO.controlMode, periodicIO.demand);
    }

    private void zeroWithHardStop() {
        setState(State.ZEROING);
        enableLimits(false);
        setOpenLoop(-0.1);
    }

    public void resetToAbsolute() {
        if (!zeroedAbsolutely) {
            if (isEncoderConnected() && RobotBase.isReal()) {
                //DriverStation.reportError("HOOD WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
                double cancoderOffset = Util.boundAngle0to360Degrees(getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle);
                double absolutePosition = Constants.MotorizedHood.kHoodStartingAngle + (cancoderOffset / Constants.MotorizedHood.kEncoderToHoodRatio);
                if (absolutePosition > Constants.MotorizedHood.kMaxInitialAngle) {
                    cancoderOffset -= 360.0;
                    absolutePosition = Constants.MotorizedHood.kHoodStartingAngle + (cancoderOffset / Constants.MotorizedHood.kEncoderToHoodRatio);
                } else if (absolutePosition < Constants.MotorizedHood.kMinInitialAngle) {
                    cancoderOffset += 360.0;
                    absolutePosition = Constants.MotorizedHood.kHoodStartingAngle + (cancoderOffset / Constants.MotorizedHood.kEncoderToHoodRatio);
                }
                if(absolutePosition > Constants.MotorizedHood.kMaxInitialAngle || absolutePosition < Constants.MotorizedHood.kMinInitialAngle){
                    DriverStation.reportError("Hood angle is out of bounds", false);
                    hasEmergency = true;
                }
                SmartDashboard.putNumber("Hood Zero", degreesToEncUnits(absolutePosition));
                hood.setSelectedSensorPosition(degreesToEncUnits(absolutePosition), 0, 0);
                //System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorizedHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle) + ");
            } else {
                DriverStation.reportError("Hood encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
                hood.setSelectedSensorPosition(degreesToEncUnits(Constants.MotorizedHood.kHoodStartingAngle), 0, 0);
            }
        }
    }

    public synchronized void zeroHood() {
        System.out.println("Hood Zeroed");
        System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorizedHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle) + ", degreesToEncUnits: " + degreesToEncUnits(getAbsoluteEncoderDegrees() - Constants.MotorizedHood.kEncStartingAngle));
        zeroedAbsolutely = true;
    }
    private void initializeDashboardValues() {
        boolean zeroedHoodValue = SmartDashboard.getBoolean("Re-zero Hood", false);
        SmartDashboard.putBoolean("Re-zero Hood", false);
    }
    private void dashboardHoodReZero() {
        SmartDashboard.putBoolean("Re-zero Hood", false);
        /*if(getState() != State.ZEROING)
            zeroHood();*/
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Angle", getAngle()); // -17 -66
        SmartDashboard.putBoolean("Hood angle is ready", hasReachedAngle());
        if(SmartDashboard.getBoolean("Re-zero Hood", false))
            dashboardHoodReZero();
        if(Settings.debugHood()) {
            SmartDashboard.putNumber("Hood Absolute Encoder", getAbsoluteEncoderDegrees()); // -281 -234
            SmartDashboard.putNumber("Hood Encoder", periodicIO.position); // -8000
            SmartDashboard.putNumber("Hood Error", encUnitsToDegrees(periodicIO.demand - periodicIO.position));
            SmartDashboard.putNumber("Hood Velocity", hood.getSelectedSensorVelocity(0));
        }
        /*if(hood.getBusVoltage() == 0)
            DriverStation.reportError("HOOD MOTOR NOT DETECTED", false);*/
    }

    public Request openLoopRequest(double percentOutput) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(percentOutput);
            }

        };
    }
    
    public Request setAngleRequest(double angle) {
        return new Request() {
            @Override
            public void act() {
                setAngleState(angle);
            }        
        };
    }
    public Request visionRequest() {
        return new Request() {
            @Override
            public void act() {
                currentState = State.VISION;
            }
        };
    }
    public Request positionStateRequest() {
        return new Request() {
            @Override
            public void act() {
                currentState = State.ROBOT_POSITION;
            }
        };
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public class PeriodicIO {
        public double position = 0.0;

        public double demand = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
    }
}