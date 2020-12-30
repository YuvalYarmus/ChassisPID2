/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class ChasisAdmin extends CommandBase implements Sendable {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chasis m_subsystem;
  private final XboxController controller; 
  private int startPos; 
  double distance_in_meters; 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChasisAdmin(Chasis subsystem, XboxController controller1) {
    m_subsystem = subsystem;
    controller = controller1; 
    // startPos = m_subsystem.getLeftSensorPos(); 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    distance_in_meters = 0; 
  
  }

  public ChasisAdmin(Chasis subsystem) {
    m_subsystem = subsystem;
    controller = null; 
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    distance_in_meters = 0; 
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_subsystem.getLeftSensorPos(); 
    SmartDashboard.putNumber("startPos", startPos); 
    m_subsystem.PIDVelocity(Constants.speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // m_subsystem.SetPower(controller.getY(Hand.kRight));
    // int pos = m_subsystem.getLeftSensorPos(); 
    // double d = Math.abs(startPos - pos);
    // System.out.println("pos is "+ pos);
    // System.out.println("pulses to meter is "+ Constants.pulsesToMeter);
    // System.out.println("d is "+ d);
    // double precPower = 100 - (10 * d / Constants.pulsesToMeter);
    // if (d > Constants.pulsesToMeter / 2) precPower = 0; 
    // m_subsystem.SetPower(precPower); 
    // SmartDashboard.putNumber("pos", pos);
    this.distance_in_meters = (m_subsystem.getLeftSensorPos() -startPos) * Constants.pulseInMeter; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.PIDVelocity(0);
    System.out.println("STOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOP");
    System.out.println("STOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOPSTOP");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(this.distance_in_meters)  > 0.8) {
      System.out.println("FinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinish");
      System.out.println("FinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinishFinish");
      return true; 
    }
    else return false; 
  }

  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method super.initSendable(builder);
    builder.addDoubleProperty("distance in meters", () -> this.distance_in_meters, null);

  }



}
