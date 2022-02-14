package com.team1323.frc2020.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team1323.frc2020.Ports;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
	private static Pigeon instance = null;
	public static Pigeon getInstance(){
		if(instance == null){
			instance = new Pigeon();
		}
		return instance;
	}
	
	private Pigeon2 pigeon/*, secondPigeon*/;

	private Rotation2d lastYaw = new Rotation2d();
    
	private Pigeon(){
		try{
			pigeon = new Pigeon2(Ports.PIGEON);
			//secondPigeon = new PigeonIMU(Ports.SECONDARY_PIGEON);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	
	public boolean isGood(PigeonIMU imu){
		return imu.getState() == PigeonState.Ready;
	}
	
	public Rotation2d getYaw(){
		if(RobotBase.isReal()){
			/*boolean firstIsGood = isGood(pigeon);
			boolean secondIsGood = isGood(secondPigeon);
			//return (lastYaw = Rotation2d.fromDegrees(pigeon.getFusedHeading()));
			if (firstIsGood && secondIsGood) {
				//System.out.println("GOT AVERAGE OF PIGEONS");
				return (lastYaw = Rotation2d.fromDegrees((pigeon.getFusedHeading() - secondPigeon.getFusedHeading()) / 2.0));
			} else if (firstIsGood) {
				//System.out.println("GOT PIGEON 1 HEADING");
				return (lastYaw = Rotation2d.fromDegrees(pigeon.getFusedHeading()));
			} else if (secondIsGood) {
				//System.out.println("GOT PIGEON 2 HEADING");
				return (lastYaw = Rotation2d.fromDegrees(-secondPigeon.getFusedHeading()));
			} else {
				//System.out.println("DIDN'T GET ANY PIGEON");
				return lastYaw;
			}*/
			return Rotation2d.fromDegrees(pigeon.getYaw());
		}
		return new Rotation2d();
	}

	public double getPitch(){
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll(){
		double [] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR(){
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}
	
	public void setAngle(double angle){
		//pigeon.setFusedHeading(angle * 64.0, 10);
		pigeon.setYaw(angle, 10);
		//secondPigeon.setFusedHeading(-angle * 64.0, 10);
		//secondPigeon.setYaw(-angle, 10);
		System.out.println("Pigeon angle set to: " + angle);
	}
	
	public void outputToSmartDashboard(){
		/*SmartDashboard.putBoolean("Pigeon 1 Good", isGood(pigeon));
		SmartDashboard.putBoolean("Pigeon 2 Good", isGood(secondPigeon));
		SmartDashboard.putNumber("Pigeon 1 Yaw", pigeon.getFusedHeading());
		SmartDashboard.putNumber("Pigeon 2 Yaw", secondPigeon.getFusedHeading());*/
	}
}
