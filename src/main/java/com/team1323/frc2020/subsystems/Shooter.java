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
    
    LazyTalonFX shooterTop, shooterBottom;
    List<LazyTalonFX> motors;
    
    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    
    public enum TopControlState {
        OPEN_LOOP, VELOCITY, VISION
    }
    public enum BottomControlState {
        OPEN_LOOP, VELOCITY, VISION
    }
    
    private TopControlState topCurrentState = TopControlState.OPEN_LOOP;
    private BottomControlState bottomCurrentState = BottomControlState.OPEN_LOOP;

    
    public TopControlState getTopState() {
        return this.topCurrentState;
    }
    public BottomControlState getBottomState() {
        return this.bottomCurrentState;
    }
    
    private double topTargetRPM = 0.0;
    private double bottomTargetRPM = 0.0;

    
    private Shooter() {
        shooterTop = new LazyTalonFX(Ports.SHOOTER_TOP);
        shooterBottom = new LazyTalonFX(Ports.SHOOTER_BOTTOM);
        

        motors = Arrays.asList(shooterTop, shooterBottom);
        
        for (LazyTalonFX motor : motors) {
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
            motor.enableVoltageCompensation(true);
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);

        }
        
        configshooterTop();
        configshooterBottom();
        setShooterOpenLoop(0.0);
    }
    private void configshooterTop() {
        shooterTop.selectProfileSlot(0, 0);
        shooterTop.config_IntegralZone(0, (int)rpmToEncVelocity(200.0, true));
        //PID Needs to be tuned
        shooterTop.config_kP(0, Constants.Shooter.kTopP);
        shooterTop.config_kI(0, Constants.Shooter.kTopI);
        shooterTop.config_kD(0, Constants.Shooter.kTopD);
        shooterTop.config_kF(0, Constants.Shooter.kTopF);
        
        shooterTop.config_kP(1, 0.15);
        shooterTop.config_kI(1, 0.0);
        shooterTop.config_kD(1, 0.0);
        shooterTop.config_kF(1, Constants.Shooter.kTopF);

        shooterTop.setInverted(TalonFXInvertType.CounterClockwise);

        shooterTop.configPeakOutputReverse(0.0, 10);
    }
    private void configshooterBottom() {
        shooterBottom.selectProfileSlot(0, 0);
        shooterBottom.config_IntegralZone(0, (int)rpmToEncVelocity(200.0, false));
        //PID Needs to be tuned
        shooterBottom.config_kP(0, Constants.Shooter.kBottomP);
        shooterBottom.config_kI(0, Constants.Shooter.kBottomI);
        shooterBottom.config_kD(0, Constants.Shooter.kBottomD);
        shooterBottom.config_kF(0, Constants.Shooter.kBottomF);
        
        shooterBottom.config_kP(1, 0.15);
        shooterBottom.config_kI(1, 0.0);
        shooterBottom.config_kD(1, 0.0);
        shooterBottom.config_kF(1, Constants.Shooter.kBottomF);

        shooterBottom.setInverted(TalonFXInvertType.CounterClockwise);

        shooterBottom.configPeakOutputReverse(0.0, 10);
    }
    public synchronized void setShooterOpenLoop(double output) {
        setBottomOpenLoop(output);
        setTopOpenLoop(output);
    }

    public synchronized void setTopOpenLoop(double output) {
        topCurrentState = TopControlState.OPEN_LOOP;
        topTargetRPM = 0;
        shooterTop.set(ControlMode.PercentOutput, output);
    }

    public synchronized void setBottomOpenLoop(double output) {
        bottomCurrentState = BottomControlState.OPEN_LOOP;
        bottomTargetRPM = 0;
        shooterBottom.set(ControlMode.PercentOutput, output);
    }

    public synchronized void setTopVelocity(double rpm) {
        topCurrentState = TopControlState.VELOCITY;
        topTargetRPM = rpm;
        shooterTop.set(ControlMode.Velocity, rpmToEncVelocity(rpm, true));
    }
    public synchronized void setBottomVelocity(double rpm) {
        bottomCurrentState = BottomControlState.VELOCITY;
        bottomTargetRPM = rpm;
        shooterBottom.set(ControlMode.Velocity, rpmToEncVelocity(rpm, false));
    }
    
    public double getTopRPM() {
        return encVelocityToRPM(shooterTop.getSelectedSensorVelocity(), true);
    }
    public double getBottomRPM() {
        return encVelocityToRPM(shooterBottom.getSelectedSensorVelocity(), false);
    }
    
    public double encVelocityToRPM(double encVelocity, boolean isTop) {
        return encVelocity / 2048.0 * 600.0 / ((isTop) ? Constants.Shooter.kTopEncToOutputRatio : Constants.Shooter.kBottomEncToOutputRatio);
    }
    
    public double rpmToEncVelocity(double RPM, boolean isTop) {
        return RPM * 2048.0 / 600.0 * ((isTop) ? Constants.Shooter.kTopEncToOutputRatio : Constants.Shooter.kBottomEncToOutputRatio);
    }
    
    public synchronized boolean hasTopReachedSetpoint() {
        return 
        getTopState() == TopControlState.VELOCITY && 
        Math.abs(topTargetRPM - getTopRPM()) < Constants.Shooter.kShooterRPMTolerance;
    }
    public synchronized boolean hasBottomReachedSetpoint() {
        return
        getBottomState() == BottomControlState.VELOCITY &&
        Math.abs(bottomTargetRPM - getBottomRPM()) < Constants.Shooter.kShooterRPMTolerance;
    }

    public Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
            switch (topCurrentState) {
                case VISION:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getAimingParameters();
                
                    if (aim.isPresent()) {
                        double range = aim.get().getRange();
                        
                        InterpolatingDouble visionTopRPM = Constants.kTopShooterTreeMap.getInterpolated(new InterpolatingDouble(range));
                        InterpolatingDouble visionBottomRPM = Constants.kBottomShooterTreeMap.getInterpolated(new InterpolatingDouble(range));
                        //InterpolatingDouble visionTopRPM = Constants.kHorizontalVelocityToTopRPM.getInterpolated(new InterpolatingDouble(aim.get().getHorizontalVelocity()));
                        //InterpolatingDouble visionBottomRPM = Constants.kHorizontalVelocityToBottomRPM.getInterpolated(new InterpolatingDouble(aim.get().getHorizontalVelocity()));

                        setTopVelocity(visionTopRPM.value);
                        setBottomVelocity(visionBottomRPM.value);
                    } else {
                        setTopVelocity(Constants.Shooter.kCloseTopRPM);
                        setBottomVelocity(Constants.Shooter.kCloseBottomRPM);
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

    public Request velocityRequest(double topRPM, double bottomRPM) {
        return new Request(){
        
            @Override
            public void act() {
                setTopVelocity(topRPM);
                setBottomVelocity(bottomRPM);
            }

            @Override
            public boolean isFinished() {
                return hasTopReachedSetpoint() && hasBottomReachedSetpoint();
            }

        };
    }

    public Request openLoopRequest(double topOutput, double bottomOutput) {
        return new Request(){
        
            @Override
            public void act() {
                setTopOpenLoop(topOutput);
                setBottomOpenLoop(bottomOutput);
            }

        };
    }
    public Request visionVelocityRequest() {
        return new Request() {

            @Override
            public void act() {
                topCurrentState = TopControlState.VISION;
                bottomCurrentState = BottomControlState.VISION;
            }

            @Override
            public boolean isFinished() {
                return hasTopReachedSetpoint() && hasBottomReachedSetpoint();
            }
        };
    }
  

 
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Top RPM", getTopRPM());
        SmartDashboard.putNumber("Shooter Bottom RPM", getBottomRPM());
        // SmartDashboard.putNumber("Shooter Master Current", periodicIO.current);
        // SmartDashboard.putNumber("Shooter Master Voltage", periodicIO.voltage);
        SmartDashboard.putNumber("Shooter Top RPM Setpoint", topTargetRPM);
        SmartDashboard.putNumber("Shooter Bottom RPM Setpoint", bottomTargetRPM);
        SmartDashboard.putBoolean("Shooter Top On Target", hasTopReachedSetpoint());
        SmartDashboard.putBoolean("Shooter Bottom On Target", hasBottomReachedSetpoint());
    }
    
    @Override
    public void stop() {
        setShooterOpenLoop(0.0);
    }

}

