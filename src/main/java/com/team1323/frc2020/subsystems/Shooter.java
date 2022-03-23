package com.team1323.frc2020.subsystems;

import java.lang.StackWalker.Option;
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
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.SmartTuner;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    
    LazyTalonFX master, slave;
    List<LazyTalonFX> motors;

    SmartTuner smartTuner;
    
    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    
    public enum State {
        OPEN_LOOP, VELOCITY, VISION, POSITION
    }
    
    private State currentState = State.OPEN_LOOP;

    public void setState(State newState) {
        currentState = newState;
    }
    
    public State getState() {
        return currentState;
    }
    
    private double targetRPM = 0.0;
    public double dashboardRPMInput = 0.0;
    
    private double limelightRange = 0.0;
    public double getTargetRange() {
        return limelightRange;
    }
    private double onTargetTimestamp = Double.POSITIVE_INFINITY;

    PeriodicIO periodicIO = new PeriodicIO();
    
    private Shooter() {
        master = new LazyTalonFX(Ports.SHOOTER_LEFT, "main"); //The leftMotor(Looking from the back of the turret)
        slave = new LazyTalonFX(Ports.SHOOTER_RIGHT, "main"); //The rightMotor(Looking from the back of the turret)

        motors = Arrays.asList(master, slave);
        
        for (LazyTalonFX motor : motors) {
            motor.setNeutralMode(NeutralMode.Coast);
            motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
            motor.enableVoltageCompensation(true);
            motor.configClosedloopRamp(0.0, Constants.kCANTimeoutMs);
            motor.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
            motor.configForwardSoftLimitThreshold(9999999);
            motor.configForwardSoftLimitEnable(false);
        }
        
        master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        master.selectProfileSlot(0, 0);
        master.config_IntegralZone(0, rpmToEncVelocity(200.0));
        master.config_kP(0, Constants.Shooter.kShooterP);
        master.config_kI(0, Constants.Shooter.kShooterI);
        master.config_kD(0, Constants.Shooter.kShooterD);
        master.config_kF(0, Constants.Shooter.kShooterF);  
        master.config_kP(1, 0.15);
        master.config_kI(1, 0.0);
        master.config_kD(1, 0.0);
        master.config_kF(1, Constants.Shooter.kShooterF);
        master.setInverted(TalonFXInvertType.CounterClockwise);
        master.configPeakOutputReverse(0.0, Constants.kCANTimeoutMs);
        master.setNeutralMode(NeutralMode.Coast);



        slave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        slave.selectProfileSlot(0, 0);
        slave.config_IntegralZone(0, rpmToEncVelocity(200.0));
        slave.config_kP(0, Constants.Shooter.kShooterP);
        slave.config_kI(0, Constants.Shooter.kShooterI);
        slave.config_kD(0, Constants.Shooter.kShooterD);
        slave.config_kF(0, Constants.Shooter.kShooterF); 
        slave.config_kP(1, 0.15);
        slave.config_kI(1, 0.0);
        slave.config_kD(1, 0.0);
        slave.config_kF(1, Constants.Shooter.kShooterF);
        slave.setInverted(TalonFXInvertType.Clockwise);
        slave.configPeakOutputReverse(0.0, Constants.kCANTimeoutMs);
        slave.setNeutralMode(NeutralMode.Coast);

        slave.set(ControlMode.Follower, Ports.SHOOTER_LEFT);
        setOpenLoop(0.0);

        smartTuner = new SmartTuner(master, "shooter");
        smartTuner.enabled(true);
        /*
        double dsRPMInputValue = SmartDashboard.getNumber("ShooterRPMInput", dashboardRPMInput);
        SmartDashboard.putNumber("ShooterRPMInput", dsRPMInputValue);*/
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
    
    public double getLeftRPM() {
        return encVelocityToRPM(periodicIO.leftVelocity);
    }
    public double getRightRPM() {
        return encVelocityToRPM(periodicIO.rightVelocity);
    }
    
    public double encVelocityToRPM(double encVelocity) {
        return encVelocity / 2048.0 * 600.0 / Constants.Shooter.kEncToOutputRatio;
    }
    
    public double rpmToEncVelocity(double rpm) {
        return rpm * 2048.0 / 600.0 * Constants.Shooter.kEncToOutputRatio;
    }
    
    public boolean hasReachedSetpoint() {
        return !Double.isInfinite(onTargetTimestamp) && (Timer.getFPGATimestamp() - onTargetTimestamp) >= Constants.Shooter.kOnTargetDuration;
    }

    public static double rpmToInitialBallVelocity(double rpm) {
        // Convert from bottom wheel RPM to motor RPM
        rpm /= Constants.Shooter.kBottomToMotorRatio;

        double bottomWheelAngularVelocity = rpm * Constants.Shooter.kBottomToMotorRatio / 60.0 * 2.0 * Math.PI; // radians/sec
        double bottomWheelEdgeVelocity = bottomWheelAngularVelocity * Constants.Shooter.kBottomWheelRadius; // inches/sec

        double topWheelAngularVelocity = rpm * Constants.Shooter.kBottomToMotorRatio * Constants.Shooter.kTopToBottomRatio / 60.0 * 2.0 * Math.PI; // radians/sec
        double topWheelEdgeVelocity = topWheelAngularVelocity * Constants.Shooter.kTopWheelRadius; // inches/sec

        return Constants.Shooter.kBallVelocityScrubFactor * (bottomWheelEdgeVelocity + topWheelEdgeVelocity) / 2.0;
    }

    // Inverse of the above
    public static double initialBallVelocityToRPM(double initial_velocity) {
        double rpm_numerator = 120.0 * initial_velocity;
        double rpm_denominator = Constants.Shooter.kBallVelocityScrubFactor * Constants.Shooter.kBottomToMotorRatio * 
                (Constants.Shooter.kBottomWheelRadius + Constants.Shooter.kTopToBottomRatio * Constants.Shooter.kTopWheelRadius) * 2.0 * Math.PI;
        double motor_rpm = rpm_numerator / rpm_denominator;
        double bottom_wheel_rpm = motor_rpm * Constants.Shooter.kBottomToMotorRatio;

        return bottom_wheel_rpm;
    }

    private static double getTimeToGoal(double vertical_velocity) {
        double determinant = Math.sqrt(vertical_velocity * vertical_velocity - (4 * -193.11 * /*-Constants.kVisionTargetRelativeHeight*/0.0));
        double time1 = (-vertical_velocity + determinant) / (2 * -193.11);
        double time2 = (-vertical_velocity - determinant) / (2 * -193.11);

        return Math.max(time1, time2);
    }

    public static double getCompensatedShooterRpm(double horizontal_velocity, double vertical_velocity) {
        double horizontal_distance = horizontal_velocity * getTimeToGoal(vertical_velocity);
        double corrected_time = Math.sqrt((Constants.kVisionTargetRelativeHeight - (Math.tan(Math.toRadians(Constants.MotorizedHood.kMinEmpiricalAngle)) * horizontal_distance)) / -193.11);
        double corrected_horizontal_velocity = horizontal_distance / corrected_time;
        double corrected_vertical_velocity = Math.tan(Math.toRadians(Constants.MotorizedHood.kMinEmpiricalAngle)) * corrected_horizontal_velocity;
        double corrected_velocity = Math.hypot(corrected_horizontal_velocity, corrected_vertical_velocity);

        return initialBallVelocityToRPM(corrected_velocity);
    }

    public Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
            boolean onTarget = getState() != State.OPEN_LOOP && Math.abs(targetRPM - getLeftRPM()) < Constants.Shooter.kShooterRPMTolerance;
            if (onTarget && Double.isInfinite(onTargetTimestamp)) {
                onTargetTimestamp = timestamp;
            } else if (!onTarget) {
                onTargetTimestamp = Double.POSITIVE_INFINITY;
            }
            switch (currentState) {
                case VISION:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getAimingParameters();
                    if (aim.isPresent()) {
                        double rpm = aim.get().getShooterRPM();
                        periodicIO.demand = rpmToEncVelocity(rpm);
                        targetRPM = rpm;
                        limelightRange = aim.get().getRange();
                    } else {
                        System.out.println("Vision target not visible in shooter loop!");
                    }
                    break;
                case POSITION:
                    Optional<ShooterAimingParameters> poseAim = RobotState.getInstance().getAimingParameters(true);
                    if (poseAim.isPresent()) {
                        double rpm = poseAim.get().getShooterRPM();
                        periodicIO.demand = rpmToEncVelocity(rpm);
                        targetRPM = rpm;
                        limelightRange = poseAim.get().getRange();
                        System.out.println("COF Magnitude" + poseAim.get().getRange());
                    } else {
                        System.out.println("Goal target not visible in shooter loop!");
                    }
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
                return true/*hasReachedSetpoint()*/;
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
    public Request positionVelocityRequest() {
        return new Request() {

            @Override
            public void act() {
                setState(State.POSITION);
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
        periodicIO.leftVelocity = master.getSelectedSensorVelocity();
        periodicIO.rightVelocity = slave.getSelectedSensorVelocity();
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
    
    public void updateRPM() {
        /*double rpmValue = SmartDashboard.getNumber("ShooterRPMInput", dashboardRPMInput);
        dashboardRPMInput = rpmValue;*/
        dashboardRPMInput = smartTuner.getValue();
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter RPM", getLeftRPM());
        SmartDashboard.putBoolean("Shooter Is Ready", hasReachedSetpoint());
        SmartDashboard.putNumber("Shooter RPM Setpoint", targetRPM);
        if(Settings.debugShooter()) {
            SmartDashboard.putNumber("Shooter Left RPM", getLeftRPM());
            SmartDashboard.putNumber("Shooter Right RPM", getRightRPM());
            SmartDashboard.putNumber("Shooter RPM Setpoint", targetRPM);
            SmartDashboard.putString("Shooter State", getState().toString());
            SmartDashboard.putNumber("Shooter Left Commanded Input", master.getMotorOutputPercent());
            SmartDashboard.putNumber("Shooter Right Commanded Input", slave.getMotorOutputPercent());
            SmartDashboard.putString("Shooter State", currentState.toString());
            SmartDashboard.putNumber("Shooter Left Current", master.getOutputCurrent());
            SmartDashboard.putNumber("Shooter Right Current", slave.getOutputCurrent());
        }
        /*if(master.getBusVoltage() == 0)
            DriverStation.reportError("LEFT SHOOTER MOTOR NOT DETECTED", false);
        if(master.getBusVoltage() == 0)
            DriverStation.reportError("RIGHT SHOOTER MOTOR NOT DETECTED", false);*/
        smartTuner.update();
        updateRPM();
        /*SmartDashboard.putNumber("Shooter Master Current", periodicIO.current);
        SmartDashboard.putBoolean("Shooter On Target", hasReachedSetpoint());*/
    }
    
    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    private static class PeriodicIO {
        // Inputs
        public double rightVelocity = 0.0;
        public double leftVelocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        // Outputs
        public double demand = 0.0;
    }
}