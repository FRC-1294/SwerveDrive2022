/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  //init swerve drive objects
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftSteer, Constants.frontLeftDrive, 
  //bruh
  new double[] {-Constants.swerveModuleXDistance, Constants.swerveModuleYDistance}, true);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.frontRightSteer, Constants.frontRightDrive, 
  new double[] {Constants.swerveModuleXDistance, Constants.swerveModuleYDistance}, true);
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.rearLeftSteer, Constants.rearLeftDrive, 
  new double[] {-Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);
  private final SwerveModule rearRightModule = new SwerveModule(Constants.rearRightSteer, Constants.rearRightDrive, 
  new double[] {Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance}, true);

  private final SwerveModule[] modules = new SwerveModule[] {frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};

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

  private final ShuffleboardTab inputTab = Shuffleboard.getTab("input");
  private final NetworkTableEntry xSpeed = inputTab.add("xSpeed", 0).getEntry();
  private final NetworkTableEntry ySpeed = inputTab.add("ySpeed", 0).getEntry();
  private final NetworkTableEntry rot = inputTab.add("rot", 0).getEntry();
  private final NetworkTableEntry angle = inputTab.add("angle", 0).getEntry();
  private final NetworkTableEntry resetEncoders = inputTab.add("reset", false).getEntry();
  private final NetworkTableEntry zeroEntry = inputTab.add("zero", false).getEntry();
  private final NetworkTableEntry resetGyro = inputTab.add("gyroReset", false).getEntry();

  private boolean zeroed = false;
  private boolean reset = true;
  private String programmingSubteam = "poopy";

  public SwerveSubsystem() {
  }

  @Override
  public void periodic() {
    
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
  }

}
