// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class RobotMap {
   
   
    //public static WPI_TalonSRX leftFollower;
    //public static WPI_TalonSRX rightFollower;
    // public static ADXRS450_Gyro gyro;
    public static WPI_TalonFX leftMaster;
    public static WPI_TalonFX rightMaster;

    public static AHRS navx;

    public static WPI_TalonFX intakeTalon;
    

    public static void init(){

        leftMaster = new WPI_TalonFX(3);
        leftMaster.configFactoryDefault();
        //leftFollower = new WPI_TalonSRX(2);
        //leftFollower.set(ControlMode.Follower, 1); 
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        leftMaster.setInverted(TalonFXInvertType.CounterClockwise);
        leftMaster.setSelectedSensorPosition(0, 0, 0);

        rightMaster = new WPI_TalonFX(1);
        leftMaster.configFactoryDefault();
        //rightFollower = new WPI_TalonSRX(4);
        //rightFollower.set(ControlMode.Follower, 3);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setInverted(TalonFXInvertType.Clockwise);

        //intakeTalon = new WPI_TalonFX(5);
        

        
        
        
        

        navx = new AHRS(SPI.Port.kMXP);
       
        


    }
}