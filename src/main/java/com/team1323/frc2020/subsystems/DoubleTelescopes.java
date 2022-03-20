// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.lib.util.SmartTuner;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DoubleTelescopes extends Subsystem {
    SmartTuner leftTuner;
    SmartTuner rightTuner;

    LazyTalonFX leftTelescope;
    LazyTalonFX rightTelescope;
    Pigeon pigeon;

    List<LazyTalonFX> motors;
    private static DoubleTelescopes instance = null;
    public static DoubleTelescopes getInstance() {
        if(instance == null)
            instance = new DoubleTelescopes();
        return instance;
    }

    public double leftTargetHeight = 0;
    public double rightTargetHeight = 0;
    private boolean liftModeEnabled = true;

    public DoubleTelescopes() {
        pigeon = Pigeon.getInstance();
        leftTelescope = new LazyTalonFX(Ports.TELESCOPE_LEFT, "main");
        rightTelescope = new LazyTalonFX(Ports.TELESCOPE_RIGHT, "main");

        motors = Arrays.asList(leftTelescope, rightTelescope);
        StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 0.1);
        for(LazyTalonFX motor : motors) {
            motor.configStatorCurrentLimit(statorLimit);
            motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
            motor.enableVoltageCompensation(true);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
            motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
            motor.setNeutralMode(NeutralMode.Brake);

            motor.configForwardSoftLimitThreshold(inchesToEncUnits(Constants.DoubleTelescopes.kMaxControlHeight), Constants.kCANTimeoutMs);
            motor.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
            
            motor.configReverseSoftLimitThreshold(inchesToEncUnits(Constants.DoubleTelescopes.kMinControlHeight), Constants.kCANTimeoutMs);
            motor.configReverseSoftLimitEnable(true);

            motor.setSelectedSensorPosition(0);
        }
        leftTuner = new SmartTuner(leftTelescope, "leftTelescope");
        leftTuner.enabled(false);
        rightTuner = new SmartTuner(rightTelescope, "rightTelescope");
        rightTuner.enabled(false);
        configPID();
    }

    private void configPID() {
        rightTelescope.config_kP(0, Constants.DoubleTelescopes.kRightP, Constants.kCANTimeoutMs);
        rightTelescope.config_kI(0, Constants.DoubleTelescopes.kRightI, Constants.kCANTimeoutMs);
        rightTelescope.config_kD(0, Constants.DoubleTelescopes.kRightD, Constants.kCANTimeoutMs);
        rightTelescope.config_kF(0, Constants.DoubleTelescopes.kRightF, Constants.kCANTimeoutMs);
        rightTelescope.selectProfileSlot(0, 0);
        rightTelescope.configMotionCruiseVelocity(Constants.DoubleTelescopes.kMaxSpeed * 1.0, Constants.kCANTimeoutMs);
        rightTelescope.configMotionAcceleration(Constants.DoubleTelescopes.kMaxSpeed * 5.0, Constants.kCANTimeoutMs);
        rightTelescope.configMotionSCurveStrength(0);
        rightTelescope.setInverted(TalonFXInvertType.Clockwise);

        leftTelescope.config_kP(0, Constants.DoubleTelescopes.kLeftP, Constants.kCANTimeoutMs);
        leftTelescope.config_kI(0, Constants.DoubleTelescopes.kLeftI, Constants.kCANTimeoutMs);
        leftTelescope.config_kD(0, Constants.DoubleTelescopes.kLeftD, Constants.kCANTimeoutMs);
        leftTelescope.config_kF(0, Constants.DoubleTelescopes.kLeftF, Constants.kCANTimeoutMs);
        leftTelescope.selectProfileSlot(0, 0);
        leftTelescope.configMotionCruiseVelocity(Constants.DoubleTelescopes.kMaxSpeed * 1.0, Constants.kCANTimeoutMs);
        leftTelescope.configMotionAcceleration(Constants.DoubleTelescopes.kMaxSpeed * 5.0, Constants.kCANTimeoutMs);
        leftTelescope.configMotionSCurveStrength(0);


    }
    PeriodicIO periodicIO = new PeriodicIO();
    public void enableLimits(boolean enable) {
        rightTelescope.configForwardSoftLimitEnable(enable);
        rightTelescope.configReverseSoftLimitEnable(enable);

        leftTelescope.configReverseSoftLimitEnable(enable);
        leftTelescope.configForwardSoftLimitEnable(enable);
    }
    static double minHeight = Constants.DoubleTelescopes.kMinControlHeight;
    static double maxHeight = Constants.DoubleTelescopes.kMaxControlHeight;
    public enum LiftMode {
        DISABLED(0, 0), START(maxHeight, maxHeight), FIRST_WINCH(minHeight, maxHeight), SECOND_INITIAL_RELEASE(minHeight + 2.0, maxHeight), SECOND_FULL_RELEASE(maxHeight, minHeight), THIRD_INITIAL_HANG(maxHeight, minHeight + 2.0), THIRD_FULL_RELEASE((maxHeight - (maxHeight / 2)),minHeight + 2.0);
        public double leftEndingHeight;
        public double rightEndingHeight;
        LiftMode(double leftTargetHeight, double rightTargetHeight) {
            leftEndingHeight = leftTargetHeight;
            rightEndingHeight = rightTargetHeight;
        }
    }
    public enum TelescopeState {
        OFF, OPEN_LOOP, POSITION;
    }
    private TelescopeState leftTelescopeState = TelescopeState.OFF;
    private TelescopeState rightTelescopeState = TelescopeState.OFF;

    public TelescopeState getRightTelescopeState() {
        return rightTelescopeState;
    }
    public TelescopeState getLeftTelescopeState() {
        return leftTelescopeState;
    }
    public void setLeftSensorPosition(double inches) {
        leftTelescope.setSelectedSensorPosition(inchesToEncUnits(inches));
    }
    public void setRightSensorPosition(double inches) {
        rightTelescope.setSelectedSensorPosition(inchesToEncUnits(inches));
    }

    private LiftMode currentLiftMode = LiftMode.DISABLED;
    public void setLiftMode(LiftMode liftMode) {
        liftModeEnabled = true;
        currentLiftMode = liftMode;
        setLeftHeight(liftMode.leftEndingHeight);
        setRightHeight(liftMode.rightEndingHeight);
    }
    public void setLeftOpenLoop(double demand) {
        leftTelescopeState = TelescopeState.OPEN_LOOP;
        periodicIO.leftDemand = demand * 0.33;
        periodicIO.leftControlMode = ControlMode.PercentOutput;
    }
    public void setRightOpenLoop(double demand) {
        rightTelescopeState = TelescopeState.OPEN_LOOP;
        periodicIO.rightDemand = demand * 0.33;
        periodicIO.rightControlMode = ControlMode.PercentOutput;
    }

    public void setLeftHeight(double heightInches) {
        leftTelescopeState = TelescopeState.POSITION;
        heightInches = Util.limit(heightInches, Constants.DoubleTelescopes.kMinControlHeight, Constants.DoubleTelescopes.kMaxControlHeight);
        periodicIO.leftDemand = inchesToEncUnits(heightInches);
        periodicIO.leftControlMode = ControlMode.MotionMagic;
        leftTargetHeight = heightInches;
    }
    public void setRightHeight(double heightInches) {
        rightTelescopeState = TelescopeState.POSITION;
        heightInches = Util.limit(heightInches, Constants.DoubleTelescopes.kMinControlHeight, Constants.DoubleTelescopes.kMaxControlHeight);
        periodicIO.rightDemand = inchesToEncUnits(heightInches);
        periodicIO.rightControlMode = ControlMode.MotionMagic;
        //System.out.println("The Right ran with a target height of :" + heightInches);

        rightTargetHeight = heightInches;
    }
    public void lockLeftHeight() {
        leftTelescopeState = TelescopeState.POSITION;
        periodicIO.leftDemand = periodicIO.leftPosition;
        periodicIO.leftControlMode = ControlMode.MotionMagic;
        leftTargetHeight = encUnitsToInches(periodicIO.leftPosition);
    }
    public void lockRightHeight() {
        rightTelescopeState = TelescopeState.POSITION;
        periodicIO.rightDemand = periodicIO.rightPosition;
        periodicIO.rightControlMode = ControlMode.MotionMagic;
        rightTargetHeight = encUnitsToInches(periodicIO.rightPosition);
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            setLeftOpenLoop(0.0);
            setRightOpenLoop(0.0);
        }

        @Override
        public void onLoop(double timestamp) {
            if(liftModeEnabled) {
                switch(currentLiftMode) {
                    case START:
                        break;
                    case FIRST_WINCH:
                        break;
                    case SECOND_INITIAL_RELEASE:
                        if(leftTelescopeOnTarget() && rightTelescopeOnTarget()) {
                            setLiftMode(LiftMode.SECOND_FULL_RELEASE);
                        }
                        break;
                    case SECOND_FULL_RELEASE:
                        break;
                    case THIRD_INITIAL_HANG:
                        if(leftTelescopeOnTarget() && rightTelescopeOnTarget()) {
                            setLiftMode(LiftMode.THIRD_FULL_RELEASE);
                        }
                        break;
                    default:
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
        }
        
    };
    public boolean leftTelescopeOnTarget() {
        return (Math.abs(leftTargetHeight - encUnitsToInches(periodicIO.leftPosition)) <= Constants.DoubleTelescopes.kHeightTolerance);
    }
    public boolean rightTelescopeOnTarget() {
        return (Math.abs(rightTargetHeight - encUnitsToInches(periodicIO.rightPosition)) <= Constants.DoubleTelescopes.kHeightTolerance);
    }
    public double inchesToEncUnits(double inches) {
        return inches * Constants.DoubleTelescopes.kTicksPerInch;
    }

    public double encUnitsToInches(double encUnits) {
        return encUnits / Constants.DoubleTelescopes.kTicksPerInch;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.rightPosition = rightTelescope.getSelectedSensorPosition();
        periodicIO.leftPosition = leftTelescope.getSelectedSensorPosition();
        if(liftModeEnabled) {
            periodicIO.rightCurrent = rightTelescope.getOutputCurrent();
            periodicIO.leftCurrent = leftTelescope.getOutputCurrent();
        }
    }
    @Override
    public void writePeriodicOutputs() {
        if(leftTelescopeState == TelescopeState.OPEN_LOOP) {
            leftTelescope.set(ControlMode.PercentOutput, periodicIO.leftDemand);
        } else if(leftTelescopeState == TelescopeState.POSITION) {
            leftTelescope.set(ControlMode.MotionMagic, periodicIO.leftDemand);
        }
        if(rightTelescopeState == TelescopeState.OPEN_LOOP) {
            rightTelescope.set(ControlMode.PercentOutput, periodicIO.rightDemand);
        } else if(rightTelescopeState == TelescopeState.POSITION) {
            rightTelescope.set(ControlMode.MotionMagic, periodicIO.rightDemand);
        }
    }
    @Override
    public void outputTelemetry() {
        rightTuner.update();
        leftTuner.update();
        SmartDashboard.putNumber("Telescope Right Height Inches", encUnitsToInches(rightTelescope.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Telescope Left Height Inches", encUnitsToInches(leftTelescope.getSelectedSensorPosition()));
        SmartDashboard.putString("Current Lift Mode", currentLiftMode.toString());
        SmartDashboard.putNumber("Telescope Right  Units", rightTelescope.getSelectedSensorPosition());
        SmartDashboard.putNumber("Telescope Left Units", leftTelescope.getSelectedSensorPosition());
        SmartDashboard.putNumber("Robot Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Robot Roll", pigeon.getRoll());
        SmartDashboard.putNumber("Robot Yaw", pigeon.getYaw().getDegrees());
        SmartDashboard.putNumber("Telescope Left Current", leftTelescope.getOutputCurrent());
        SmartDashboard.putNumber("Telescope Right Current", rightTelescope.getOutputCurrent());
        SmartDashboard.putNumber("Telescope Left Target", leftTargetHeight);
        SmartDashboard.putNumber("Telescope Right Target", rightTargetHeight);
        SmartDashboard.putString("Telescope Right State", rightTelescopeState.toString());
        SmartDashboard.putString("Telescope Left State", leftTelescopeState.toString());
        SmartDashboard.putBoolean("Telescope Left On Target", leftTelescopeOnTarget());
        SmartDashboard.putBoolean("Telescope Right On Target", rightTelescopeOnTarget());
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }


    @Override
    public void stop() {
        liftModeEnabled = false;
        setLeftOpenLoop(0.0);
        setRightOpenLoop(0.0);
    }
    public static class PeriodicIO {
        public double rightDemand = 0;
        public ControlMode rightControlMode = ControlMode.PercentOutput;
        public double rightPosition = 0;
        public double rightCurrent = 0;

        public double leftDemand = 0;
        public ControlMode leftControlMode = ControlMode.PercentOutput;
        public double leftPosition = 0;
        public double leftCurrent = 0;
    }
}
