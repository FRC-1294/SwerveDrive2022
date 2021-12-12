/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  //init swerve drive objects
  // private final SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftSteer, Constants.frontLeftDrive, new double[] {-Constants.swerveModuleXDistance, Constants.swerveModuleYDistance});
  private final SwerveModule frontRightModule = new SwerveModule(Constants.frontRightSteer, Constants.frontRightDrive, 
  new double[] {Constants.swerveModuleXDistance, Constants.swerveModuleYDistance}, true);
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.rearLeftSteer, Constants.rearLeftDrive, 
  new double[] {-Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);
  // private final SwerveModule rearRightModule = new SwerveModule(Constants.rearRightSteer, Constants.rearRightDrive, 
  // new double[] {Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);

  //init gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  //init joysticks
  private final XboxController driveController = new XboxController(0);

  //init network tables
  private final NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve Data");
  private final NetworkTableEntry frontLeftStateEntry = swerveTable.getEntry("frontLeftState");
  private final NetworkTableEntry frontRightStateEntry = swerveTable.getEntry("frontRightState");
  private final NetworkTableEntry backLeftStateEntry = swerveTable.getEntry("backLeftState");
  private final NetworkTableEntry backRightStateEntry = swerveTable.getEntry("backRightState");

  public SwerveSubsystem() {
    frontRightModule.init();
    rearLeftModule.init();
    gyro.reset();
  }

  @Override
  public void periodic() {
    //gets joystick values
    double xSpeed = driveController.getX(Hand.kLeft);
    double ySpeed = -driveController.getY(Hand.kLeft);
    double rot = (driveController.getTriggerAxis(Hand.kLeft)-driveController.getTriggerAxis(Hand.kRight));

    drive(xSpeed, ySpeed, rot, true);
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //calculates speed and angle of modules and sets states
    // frontLeftModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    frontRightModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    rearLeftModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    // rearRightModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());

    // frontLeftStateEntry.setDoubleArray(new double [] {frontLeftModule.getSetVelocity(), frontLeftModule.getSetAngle()});
    frontRightStateEntry.setDoubleArray(new double [] {frontRightModule.getSetAngle(), frontRightModule.getCurrentAngle()});
    backLeftStateEntry.setDoubleArray(new double [] {rearLeftModule.getSetAngle(), rearLeftModule.getCurrentAngle()});
    // backRightStateEntry.setDoubleArray(new double [] {rearRightModule.getSetVelocity(), rearRightModule.getSetAngle()});
  }

  public void zero() {
    frontRightModule.zero();
  }
}
