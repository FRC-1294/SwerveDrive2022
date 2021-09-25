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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  //init swerve drive objects
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftSteer, Constants.frontLeftDrive, new double[] {-Constants.swerveModuleXDistance, Constants.swerveModuleYDistance});
  private final SwerveModule frontRightModule = new SwerveModule(Constants.frontRightSteer, Constants.frontRightDrive, new double[] {Constants.swerveModuleXDistance, Constants.swerveModuleYDistance});
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.rearLeftSteer, Constants.rearLeftDrive, new double[] {-Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance});
  private final SwerveModule rearRightModule = new SwerveModule(Constants.rearRightSteer, Constants.rearRightDrive, new double[] {Constants.swerveModuleXDistance, -Constants.swerveModuleYDistance});

  //init gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  //init joysticks
  private final XboxController driveController = new XboxController(0);

  //init network tables
  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("dataTable");
  private final NetworkTableEntry frontLeftStateEntry = networkTable.getEntry("frontLeftState");
  private final NetworkTableEntry frontRightStateEntry = networkTable.getEntry("frontRightState");
  private final NetworkTableEntry backLeftStateEntry = networkTable.getEntry("backLeftState");
  private final NetworkTableEntry backRightStateEntry = networkTable.getEntry("backRightState");

  public SwerveSubsystem() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    double xSpeed = driveController.getX(Hand.kLeft);
    double ySpeed = -driveController.getY(Hand.kLeft);
    double rot = (driveController.getTriggerAxis(Hand.kLeft)-driveController.getTriggerAxis(Hand.kRight));

    drive(xSpeed, ySpeed, rot, true);
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //sets current speed of the robot
    frontLeftModule.setModuleState(xSpeed, ySpeed, rot, fieldRelative);
    frontRightModule.setModuleState(xSpeed, ySpeed, rot, fieldRelative);
    rearLeftModule.setModuleState(xSpeed, ySpeed, rot, fieldRelative);
    rearRightModule.setModuleState(xSpeed, ySpeed, rot, fieldRelative);

    frontLeftStateEntry.setDoubleArray(new double [] {frontLeftModule.getSetVelocity(), frontLeftModule.getSetAngle()});
    frontRightStateEntry.setDoubleArray(new double [] {frontRightModule.getSetVelocity(), frontRightModule.getSetAngle()});
    backLeftStateEntry.setDoubleArray(new double [] {rearLeftModule.getSetVelocity(), rearLeftModule.getSetAngle()});
    backRightStateEntry.setDoubleArray(new double [] {rearRightModule.getSetVelocity(), rearRightModule.getSetAngle()});
  }
}
