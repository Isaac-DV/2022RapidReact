package com.team1323.frc2020.auto;

import com.team1323.frc2020.auto.modes.FiveBallOneEjectMode;
import com.team1323.frc2020.auto.modes.SixBallOneEjectMode;
import com.team1323.frc2020.auto.modes.StandStillMode;
import com.team1323.frc2020.auto.modes.TaxiOneBallMode;
import com.team1323.frc2020.auto.modes.TestMode;
import com.team1323.frc2020.auto.modes.ThreeBallPoachBlueAllianceMode;
import com.team1323.frc2020.auto.modes.TwoBallBackHubHideMode;
import com.team1323.frc2020.auto.modes.TwoBallCloseHideMode;
import com.team1323.frc2020.auto.modes.TwoBallTwoEjectMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.FIVE_BALL_ONE_EJECT_MODE;

    private SendableChooser<AutoOption> modeChooser;

    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        //modeChooser.addOption(AutoOption.TEST_MODE.name, AutoOption.TEST_MODE);
        modeChooser.addOption(AutoOption.STAND_STILL.name, AutoOption.STAND_STILL);
        modeChooser.addOption(AutoOption.TAXI_ONE_BALL_MODE.name, AutoOption.TAXI_ONE_BALL_MODE);
        modeChooser.addOption(AutoOption.TWO_BALL_TWO_EJECT_MODE.name, AutoOption.TWO_BALL_TWO_EJECT_MODE);
        modeChooser.addOption(AutoOption.THREE_BALL_POACH_MODE.name, AutoOption.THREE_BALL_POACH_MODE);
        modeChooser.addOption(AutoOption.TWO_BALL_HUB_HIDE_MODE.name, AutoOption.TWO_BALL_HUB_HIDE_MODE);
        modeChooser.addOption(AutoOption.TWO_BALL_CLOSE_HIDE_MIDE.name, AutoOption.TWO_BALL_CLOSE_HIDE_MIDE);
        
        SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();                
        return createAutoMode(selectedOption);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    enum AutoOption{
        STAND_STILL("Stand Still"), TEST_MODE("Test Mode"),
        FIVE_BALL_ONE_EJECT_MODE("Five Ball One Eject Mode"),
        TWO_BALL_TWO_EJECT_MODE("Two Ball Two Eject Mode"),
        TAXI_ONE_BALL_MODE("Taxi One Ball Mode"),
        THREE_BALL_POACH_MODE("Three Ball Poach Mode"),
        TWO_BALL_HUB_HIDE_MODE("Two Ball Back Hub Hide Mode"),
        TWO_BALL_CLOSE_HIDE_MIDE("Two Ball Close Hide Mode");

    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option){
    	switch(option){
            case STAND_STILL:
                return new StandStillMode();
            case TEST_MODE:
                return new TestMode();
            case FIVE_BALL_ONE_EJECT_MODE:
                return new FiveBallOneEjectMode();
            case TWO_BALL_TWO_EJECT_MODE:
                return new TwoBallTwoEjectMode();
            case TAXI_ONE_BALL_MODE:
                return new TaxiOneBallMode();
            case THREE_BALL_POACH_MODE:
                return new ThreeBallPoachBlueAllianceMode();
            case TWO_BALL_HUB_HIDE_MODE:
                return new TwoBallBackHubHideMode();
            case TWO_BALL_CLOSE_HIDE_MIDE:
                return new TwoBallCloseHideMode();
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
