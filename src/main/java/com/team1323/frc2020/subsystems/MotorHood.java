
package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.SmartTuning;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An adjustable shooter hood, powered by a BAG motor
 */
public class MotorHood extends Subsystem {
    SmartTuning smartTuner;

    private static MotorHood instance = null;
    public static MotorHood getInstance() {
        if (instance == null)
            instance = new MotorHood();
        return instance;
    }

    LazyTalonFX hood;
    DutyCycle encoder;
    RobotState robotState;
    
    public double angleInput = 0;
    private boolean isEncoderFlipped = false;
    private boolean zeroedAbsolutely = false;

    public PeriodicIO periodicIO = new PeriodicIO();

    public MotorHood() {
        hood = new LazyTalonFX(Ports.HOOD_TALON, "main");
        encoder = new DutyCycle(new DigitalInput(Ports.HOOD_ENCODER));

        hood.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        hood.enableVoltageCompensation(true);
        hood.setInverted(TalonFXInvertType.Clockwise);
        setEncoderPhase(false);

        hood.setNeutralMode(NeutralMode.Coast);

        hood.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        setCurrentLimit(10.0);
 
        hood.selectProfileSlot(0, 0);
        hood.config_kP(0, Constants.MotorHood.kP, Constants.kCANTimeoutMs);
        hood.config_kI(0, Constants.MotorHood.kI, Constants.kCANTimeoutMs);
        hood.config_kD(0, Constants.MotorHood.kD, Constants.kCANTimeoutMs);
        hood.config_kF(0, Constants.MotorHood.kF, Constants.kCANTimeoutMs);

        hood.configMotionCruiseVelocity(Constants.MotorHood.kMaxSpeed * 1.0, Constants.kCANTimeoutMs);
        hood.configMotionAcceleration(Constants.MotorHood.kMaxSpeed * 10.0, Constants.kCANTimeoutMs);

        hood.configForwardSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorHood.kMaxControlAngle), Constants.kCANTimeoutMs);
        hood.configReverseSoftLimitThreshold(hoodAngleToEncUnits(Constants.MotorHood.kMinControlAngle), Constants.kCANTimeoutMs);
        hood.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        hood.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);

        hood.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        hood.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);
        
        hood.setSelectedSensorPosition(degreesToEncUnits(15.0));
        resetToAbsolute();
        //hood.setSelectedSensorPosition(0);

        setOpenLoop(0.0);
        smartTuner = new SmartTuning(hood, "hood");
        smartTuner.enabled(true);
    }

    private void setCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, amps, amps, 10);
        hood.configSupplyCurrentLimit(currentLimitConfiguration, Constants.kCANTimeoutMs);
    }

    private void setEncoderPhase(boolean phase) {
        isEncoderFlipped = phase;
    }

    private double getAbsoluteEncoderDegrees() {
        return (isEncoderFlipped ? -1.0 : 1.0) * encoder.getOutput() * 360.0;
    }

    private boolean isEncoderConnected() {
        if (RobotBase.isReal()) {
            return encoder.getFrequency() != 0;
        }
        return true;
    }

    public double encUnitsToDegrees(double encUnits) {
        return encUnits / Constants.MotorHood.kTicksPerDegree;
    }

    public double degreesToEncUnits(double degrees) {
        return (degrees * Constants.MotorHood.kTicksPerDegree);
    }

    public double encUnitsToHoodAngle(double encUnits) {
        return encUnitsToDegrees(encUnits);
    }

    public double hoodAngleToEncUnits(double degrees) {
        return (degreesToEncUnits(degrees));
    }

    public double getAngle() {
        return encUnitsToHoodAngle(periodicIO.position);
    }

    public void setAngle(double angle) {
        angle = Util.limit(angle, Constants.MotorHood.kMinControlAngle, Constants.MotorHood.kMaxControlAngle);

        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.demand = hoodAngleToEncUnits(angle);
    }

    public void lockAngle() {
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.demand = periodicIO.position;
    }

    public boolean hasReachedAngle() {
        return periodicIO.controlMode == ControlMode.MotionMagic && 
            Math.abs(encUnitsToDegrees(periodicIO.demand - periodicIO.position)) <= Constants.MotorHood.kAngleTolerance;
    }

    public void setOpenLoop(double percentOutput) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = percentOutput;
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            hood.setNeutralMode(NeutralMode.Brake);            
        }

        @Override
        public void onLoop(double timestamp) {
            smartTuner.update();   
            angleInput = smartTuner.getValue();         
        }

        @Override
        public void onStop(double timestamp) {
            hood.setNeutralMode(NeutralMode.Coast);
        }
        
    };

    public Request angleRequest(double angle) {
        return new Request(){
        
            @Override
            public void act() {
                setAngle(angle);
            }

            @Override
            public boolean isFinished() {
                return hasReachedAngle();
            }
            
        };
    }

    public Request openLoopRequest(double percentOutput) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(percentOutput);
            }

        };
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = hood.getSelectedSensorPosition(0);
    }

    @Override
    public void writePeriodicOutputs() {
        hood.set(periodicIO.controlMode, periodicIO.demand);
    }

    public void resetToAbsolute() {
        if (!zeroedAbsolutely) {
            if (isEncoderConnected() && RobotBase.isReal()) {
                //DriverStation.reportError("HOOD WAS RESET TO ABSOLUTE WITH THE MAG ENCODER", false);
                double cancoderOffset = Util.boundAngle0to360Degrees(getAbsoluteEncoderDegrees() - Constants.MotorHood.kEncStartingAngle);
                double absolutePosition = Constants.MotorHood.kHoodStartingAngle + (cancoderOffset / Constants.MotorHood.kEncoderToHoodRatio);
                if (absolutePosition > Constants.MotorHood.kMaxInitialAngle)
                    absolutePosition -= 360.0;
                else if (absolutePosition < Constants.MotorHood.kMinInitialAngle)
                    absolutePosition += 360.0;
                if(absolutePosition > Constants.MotorHood.kMaxInitialAngle || absolutePosition < Constants.MotorHood.kMinInitialAngle){
                    DriverStation.reportError("Hood angle is out of bounds", false);
                    hasEmergency = true;
                }
                SmartDashboard.putNumber("Hood Zero", degreesToEncUnits(absolutePosition));
                hood.setSelectedSensorPosition(degreesToEncUnits(absolutePosition), 0, Constants.kCANTimeoutMs);
                //System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorHood.kEncStartingAngle) + ");
            } else {
                DriverStation.reportError("Hood encoder NOT DETECTED: CURRENT POSITION SET TO 0", false);
                hood.setSelectedSensorPosition(degreesToEncUnits(Constants.MotorHood.kHoodStartingAngle), 0, Constants.kCANTimeoutMs);
            }
        }
    }

    public synchronized void zeroHood() {
        System.out.println("Hood Zeroed");
        System.out.println("Hood Absolute angle: " + getAbsoluteEncoderDegrees() + ", encoder offset: " + Constants.MotorHood.kEncStartingAngle + ", difference: " + (getAbsoluteEncoderDegrees() - Constants.MotorHood.kEncStartingAngle) + ", degreesToEncUnits: " + degreesToEncUnits(getAbsoluteEncoderDegrees() - Constants.MotorHood.kEncStartingAngle));
        zeroedAbsolutely = true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Absolute Encoder", getAbsoluteEncoderDegrees()); // -281 -234
        //SmartDashboard.putNumber("Hood Encoder", periodicIO.position); // -8000
        SmartDashboard.putNumber("Hood Angle", getAngle()); // -17 -66
        //SmartDashboard.putNumber("Hood Error", encUnitsToDegrees(periodicIO.demand - periodicIO.position));
        //SmartDashboard.putNumber("Hood Velocity", hood.getSelectedSensorVelocity(0));
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