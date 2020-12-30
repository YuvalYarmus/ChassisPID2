/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int BackRightID=1;
    public static final int BackLeftID=4;
    public static final int FrontRightID=2;
    public static final int FrontLeftID=3;

    public static final double CheesyPuffsMeter = 2.54 / 100.; 
    public static final int wheelD = 6; ///in inches
    public static final double  D = wheelD * CheesyPuffsMeter; // in meters 

    public static final double P = D * Math.PI ; // wheel paremeter 
    public static final double pulses = 800; 
    // public static final double pulsesToMeter = (20/ Constants.D) * Constants.pulses; 
    public static final double pulsesToMeter = (1/Constants.P) * Constants.pulses; 

    public static final double pulseInMeter = P / pulses; 
    public static final double speed = 0.4; // meters in seconds 
    public static final double pulseToSpeed = speed / pulseInMeter; // pulses in the amount of meters per sec
  

    public static final double pulsesin10thsec = pulseToSpeed / 10; // pulses in the amount of meters per sec

    // public static final double kp = 0.00193;
    // public static final double kd = kp / 10; 
    // public static final double ki = 0; 
    // public static final double ks = 1.38; 
    // public static final double kv = 2.55; 
    // public static final double ka = 0.149; 
    
    public static final double kp = 0.0001;
    public static final double kd = 0; 
    public static final double ki = 0; 
    public static final double ks = 0.797; 
    public static final double kv = 2.34; 
    public static final double ka = 0.862; 
    
 



}
    