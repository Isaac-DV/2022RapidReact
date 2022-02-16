package com.team1323.frc2020.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    
    LazyTalonFX master;
    List<LazyTalonFX> motors;
    
    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    
    public enum State {
        OPEN_LOOP, VELOCITY, VISION
    }
    
    private State currentState = State.OPEN_LOOP;

    private void setState(State newState) {
        currentState = newState;
    }
    
    public State getState() {
        return currentState;
    }
    
    private double targetRPM = 0.0;

    PeriodicIO periodicIO = new PeriodicIO();
    
    private Shooter() {
        master = new LazyTalonFX(Ports.SHOOTER_BOTTOM, "main");

        motors = Arrays.asList(master);
        
        for (LazyTalonFX motor : motors) {
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
            motor.enableVoltageCompensation(true);
        }
        
        master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        master.selectProfileSlot(0, 0);
        master.config_IntegralZone(0, (int)rpmToEncVelocity(200.0));
        //PID Needs to be tuned
        master.config_kP(0, Constants.Shooter.kShooterP);
        master.config_kI(0, Constants.Shooter.kShooterI);
        master.config_kD(0, Constants.Shooter.kShooterD);
        master.config_kF(0, Constants.Shooter.kShooterF);
        
        master.config_kP(1, 0.15);
        master.config_kI(1, 0.0);
        master.config_kD(1, 0.0);
        master.config_kF(1, Constants.Shooter.kShooterF);

        master.setInverted(TalonFXInvertType.CounterClockwise);

        master.configPeakOutputReverse(0.0, 10);
        
        setOpenLoop(0.0);
    }

    public void setOpenLoop(double output) {
        setState(State.OPEN_LOOP);
        periodicIO.demand = output;
        targetRPM = 0;
    }

    public void setVelocity(double rpm) {
        setState(State.VELOCITY);
        periodicIO.demand = rpmToEncVelocity(rpm);
        targetRPM = rpm;
    }
    
    public double getRPM() {
        return encVelocityToRPM(periodicIO.velocity);
    }
    
    public double encVelocityToRPM(double encVelocity) {
        return encVelocity / 2048.0 * 600.0 / Constants.Shooter.kEncToOutputRatio;
    }
    
    public double rpmToEncVelocity(double RPM) {
        return RPM * 2048.0 / 600.0 * Constants.Shooter.kEncToOutputRatio;
    }
    
    public  boolean hasReachedSetpoint() {
        return getState() == State.VELOCITY && Math.abs(targetRPM - getRPM()) < Constants.Shooter.kShooterRPMTolerance;
    }

    public Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
            switch (currentState) {
                case VISION:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getAimingParameters();
                    if (aim.isPresent()) {
                        InterpolatingDouble visionRPM = Constants.kHorizontalVelocityToBottomRPM.getInterpolated(new InterpolatingDouble(aim.get().getHorizontalVelocity()));
                        setVelocity(visionRPM.value);
                    } else {
                        setVelocity(Constants.Shooter.kCloseBottomRPM);
                        System.out.println("Vision target not visible in shooter loop!");
                    }
                    break;
                default:
                    break;
            }
        }
        

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    public Request velocityRequest(double rpm) {
        return new Request(){
        
            @Override
            public void act() {
                setVelocity(rpm);
            }

            @Override
            public boolean isFinished() {
                return hasReachedSetpoint();
            }

        };
    }

    public Request visionVelocityRequest() {
        return new Request() {

            @Override
            public void act() {
                setState(State.VISION);
            }

            @Override
            public boolean isFinished() {
                return hasReachedSetpoint();
            }
        };
    }

    public Request waitForBallRequest() {
        return new Request() {
            
            @Override
            public void act() {

            }

            @Override
            public boolean isFinished() {
                return !hasReachedSetpoint();
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

    @Override
    public void readPeriodicInputs() {
        periodicIO.velocity = master.getSelectedSensorVelocity();
        periodicIO.current = master.getStatorCurrent();
    }

    @Override
    public void writePeriodicOutputs() {
        if (getState() == State.VELOCITY || getState() == State.VISION) {
            master.set(ControlMode.Velocity, periodicIO.demand);
        } else {
            master.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        /*SmartDashboard.putNumber("Shooter Master Current", periodicIO.current);
        SmartDashboard.putNumber("Shooter RPM Setpoint", targetRPM);
        SmartDashboard.putBoolean("Shooter On Target", hasReachedSetpoint());*/
    }
    
    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    private static class PeriodicIO {
        // Inputs
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        // Outputs
        public double demand = 0.0;
    }
}