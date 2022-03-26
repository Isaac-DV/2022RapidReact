package com.team1323.frc2020.auto;

import com.team1323.frc2020.auto.modes.FiveBallOneEjectMode;
import com.team1323.frc2020.auto.modes.SixBallOneEjectMode;
import com.team1323.frc2020.auto.modes.StandStillMode;
import com.team1323.frc2020.auto.modes.TestMode;
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
        modeChooser.addOption(AutoOption.TWO_BALL_TWO_EJECT_MODE.name, AutoOption.TWO_BALL_TWO_EJECT_MODE);
        modeChooser.addOption(AutoOption.STAND_STILL.name, AutoOption.STAND_STILL);

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
        TWO_BALL_TWO_EJECT_MODE("Two Ball Two Eject Mode");
    	
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
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
