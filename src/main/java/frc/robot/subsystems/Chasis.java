/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chasis extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public TalonSRX BackLeft = new TalonSRX(Constants.BackLeftID);
  public TalonSRX BackRight = new TalonSRX(Constants.BackRightID);
  public TalonSRX FrontLeft = new TalonSRX(Constants.FrontLeftID);
  public TalonSRX FrontRight = new TalonSRX(Constants.FrontRightID);
  private double vel;

  public Chasis() {
    SmartDashboard.putData(this);
    BackRight.follow(FrontRight);
    FrontLeft.follow(BackLeft);

    BackLeft.config_kP(0, Constants.kp);
    FrontLeft.config_kP(0, Constants.kp);
    FrontRight.config_kP(0, Constants.kp);
    BackRight.config_kP(0, Constants.kp);
    vel = 0; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void SetPower(double power) {
    power = power / 500;
    System.out.println("power is " + power);
    BackLeft.set(ControlMode.PercentOutput, -power);
    BackRight.set(ControlMode.PercentOutput, -power);
    FrontLeft.set(ControlMode.PercentOutput, -power);
    FrontRight.set(ControlMode.PercentOutput, -power);
    // FrontLeft.getSelectedSensorPosition();
  }

  public void PIDVelocity(double vel) {

    // BackLeft.set(ControlMode.Velocity, 0);
    // BackLeft.set(Mode, demand0, demand1Type, demand1);
    this.vel = vel; 
    
    if (vel == 0) {
      BackLeft.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward,
        0);

    FrontRight.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward,
        0);

    }
    else {
      double velPulse = vel / Constants.pulseInMeter/ 10; 
      BackLeft.set(ControlMode.Velocity, -velPulse, DemandType.ArbitraryFeedForward,
          (vel * Constants.ks + Constants.kv) / 12);
  
      FrontRight.set(ControlMode.Velocity, -velPulse, DemandType.ArbitraryFeedForward,
          (vel * Constants.ks + Constants.kv) / 12);
  
    }

  }

  public double get_ks() {
    return Constants.ks;
  }

  public double get_kv() {
    return Constants.kv;
  }

  public double get_kp() {
    return Constants.kp;
  }

  public double pulsesin10thsec() {
    return Constants.pulsesin10thsec;
  }

  public int getLeftSensorPos() {
    // System.err.println( FrontLeft.getSelectedSensorPosition());

    return FrontLeft.getSelectedSensorPosition();
  }

  public double Speed1() {
    return FrontLeft.getSelectedSensorVelocity();
  }

  public double SpeedInMtoSec1() {
    return this.Speed1() * 10 / Constants.pulsesToMeter;
  }

  public double Speed2() {
    return FrontRight.getSelectedSensorVelocity();
  }

  public double SpeedInMtoSec2() {
    return this.Speed2() * 10 / Constants.pulsesToMeter;
  }

  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method super.initSendable(builder);
    builder.addDoubleProperty("SET VEL", () -> this.vel, null);
    builder.addDoubleProperty("left encoder", this::getLeftSensorPos, null);
    builder.addDoubleProperty("ks", this::get_ks, null);
    builder.addDoubleProperty("kv", this::get_kv, null);
    builder.addDoubleProperty("kp", this::get_kp, null);
    builder.addDoubleProperty("pulsesin10thsec", this::pulsesin10thsec, null);
    builder.addDoubleProperty("Left encoder speed", this::Speed1, null);
    // builder.addDoubleProperty("Left speed", () -> this:: SpeedInMtoSec1, null);
    builder.addDoubleProperty("Left speed", this::SpeedInMtoSec1, null);
    builder.addDoubleProperty("Right encoder speed", this::Speed2, null);
    builder.addDoubleProperty("Right speed", this::SpeedInMtoSec2, null);
    builder.addDoubleProperty("left closed error", BackLeft::getClosedLoopError, null);
    builder.addDoubleProperty("left output voltage", BackLeft::getMotorOutputVoltage, null);
    builder.addDoubleProperty("left closed error", FrontRight::getClosedLoopError, null);
    builder.addDoubleProperty("left output voltage", FrontRight::getMotorOutputVoltage, null);
  }

  public int getRightSensorPos() {
    return BackRight.getSelectedSensorPosition();
  }

}
