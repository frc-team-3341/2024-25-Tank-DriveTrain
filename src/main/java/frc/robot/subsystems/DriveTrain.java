// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon; 
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP); //This is creating a new object within the AHRS class called navx

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //This titles the variable DTTab as DriveTrain.
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry(); //updates DTTab with LeftVoltage with a value of 0.0, and stores the return value of getEntry into LeftVoltage
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry(); //updates DTTab with RightVoltage with a value of 0.0, and stores the return value of getEntry into RighthVoltage

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //This sets up the leftDriveTalon
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort); //This sets up the rightDriveTalon
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //This makes sure that the leftDriveTalon is set to neutral
    rightDriveTalon.setNeutralMode(NeutralMode.Coast); //This makes sure that the rightDriveTalon is set to neutral

    leftDriveTalon.setInverted(true); //This makes sure the leftDriveTalon is inverted in perspective to the rightDriveTalon so it can drive forward and backward properly
    rightDriveTalon.setInverted(true); //This makes sure the rightDriveTalon is inverted in perspective to the leftDriveTalon so it can drive forward and backward properly

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault(); //resets leftDriveTalon
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //This sets up the encoder to track rotation of the motor
    rightDriveTalon.configFactoryDefault(); //resets rightDriveTalon
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {  //This will drive the robot with a certain speed
    rightDriveTalon.set(rightSpeed); 
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10); //Sets the sensor position of the leftDriveTalon to 0, 0, 10
    rightDriveTalon.setSelectedSensorPosition(0,0,10); //Sets the sensor position of the rightDriveTalon to 0, 0, 10
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
  public double getMeters(){
    return (Units.inchesToMeters(6)*Math.PI /4096*getTicks());
  }  
  public double getAngle(){ //Gets the robot's current angle
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}
