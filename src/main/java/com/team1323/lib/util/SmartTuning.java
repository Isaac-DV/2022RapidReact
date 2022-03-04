// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import com.team1323.frc2020.Constants;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SmartTuning {

    private LazyTalonFX motor;
    private String keyName;

    private boolean enabled = false;

    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double mValue;
    public double getValue() {
        return mValue;
    }
    //private CreatedDashboardValue[] mDBValuesArray;



    public SmartTuning(LazyTalonFX motorInstance, String dashboardKeyName) {
        motor = motorInstance;
        keyName = dashboardKeyName;
        initalizeWidgets();
    }
    public SmartTuning(String dashboardKeyName) {
        keyName = dashboardKeyName;
        createDashboardKey(keyName + "Input", 0);
    }
    public void initalizeWidgets() {
        createDashboardKey(keyName + "kP", 0);
        createDashboardKey(keyName + "kI", 0);
        createDashboardKey(keyName + "kD", 0);
        createDashboardKey(keyName + "kF", 0);
        createDashboardKey(keyName + "Input", 0);
    }

    public void update() {
        if(this.enabled) {
            updatekP(getDashboardNumberValue(keyName + "kP"));
            updatekI(getDashboardNumberValue(keyName + "kI"));
            updatekD(getDashboardNumberValue(keyName + "kD"));
            updatekF(getDashboardNumberValue(keyName + "kF"));
            mValue = getDashboardNumberValue(keyName + "Input");
            /*for(int i = 0; i < mDBValuesArray.length; i++) {
                CreatedDashboardValue currentDBValue = mDBValuesArray[i];
                if(currentDBValue.valueType == 's') {
                    currentDBValue.valueString = getDashboardStringValue(currentDBValue.key);
                } else if(currentDBValue.valueType == 'n') {
                    currentDBValue.valueNumber = getDashboardNumberValue(currentDBValue.key);
                }
            }*/
        }
    }
    
    
    public void createDashboardKey(String keyName, String value) {
        value = getDashboardStringValue(keyName);
        SmartDashboard.putString(keyName, value);
    }
    public void createDashboardKey(String keyName, double value) {
        value = getDashboardNumberValue(keyName);
        SmartDashboard.putNumber(keyName, value);
    }

    /*public Object getDashboardValue(String keyName) {
        for(int i = 0; i < mDBValuesArray.length; i++) {
            CreatedDashboardValue currentDBValue = mDBValuesArray[i];
            if(currentDBValue.key == keyName) {
                if(currentDBValue.valueType == 's') {
                    return (String) (currentDBValue.valueString);
                } else if(currentDBValue.valueType == 'n') {
                    return (double) (currentDBValue.valueNumber);
                }
            }
        }
        return null;
    }*/
    public double getDashboardNumberValue(String keyName) {
        return SmartDashboard.getNumber(keyName, 0);
    }
    public String getDashboardStringValue(String keyName) {
        return SmartDashboard.getString(keyName, "");
    }



    public void updatekP(double value) {
        if(kP != value) {
            kP = value;
            motor.config_kP(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekI(double value) {
        if(kI != value) {
            kI = value;
            motor.config_kI(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekD(double value) {
        if(kD != value) {
            kD = value;
            motor.config_kD(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekF(double value) {
        if(kF != value) {
            kF = value;
            motor.config_kF(0, value, Constants.kCANTimeoutMs);
        }
    }

    public void enabled(boolean enabled) {
        this.enabled = enabled;
    }

    /*private class CreatedDashboardValue {
        public String key;
        public String valueString;
        public double valueNumber;
        public char valueType;
        public CreatedDashboardValue(String key, String value) {
            this.key = key;
            this.valueString = value;
            this.valueType = 's';
            addToArray();
        }
        public CreatedDashboardValue(String key, double value) {
            this.key = key;
            this.valueNumber = value;
            this.valueType = 'n';
            addToArray();
        }
        protected void addToArray() {
            CreatedDashboardValue[] newArray = new CreatedDashboardValue[mDBValuesArray.length + 1];
            for(int i = 0; mDBValuesArray.length > i; i++) {
                newArray[i] = mDBValuesArray[i];
            }
            newArray[mDBValuesArray.length] = this;
            mDBValuesArray = newArray;
        }
        
    }*/
}
