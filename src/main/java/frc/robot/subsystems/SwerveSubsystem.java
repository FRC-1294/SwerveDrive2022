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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private final ShuffleboardTab inputTab = Shuffleboard.getTab("input");
  private final NetworkTableEntry xSpeed = inputTab.add("xSpeed", 0).getEntry();
  private final NetworkTableEntry ySpeed = inputTab.add("ySpeed", 0).getEntry();
  private final NetworkTableEntry rot = inputTab.add("rot", 0).getEntry();
  private final NetworkTableEntry angle = inputTab.add("angle", 0).getEntry();
  private final NetworkTableEntry resetEncoders = inputTab.add("reset", false).getEntry();
  private final NetworkTableEntry zeroEntry = inputTab.add("zero", false).getEntry();

  public SwerveSubsystem() {
    frontRightModule.init();
    rearLeftModule.init();
    gyro.reset();
  }

  @Override
  public void periodic() {
    //gets joystick values
    // double xSpeed = driveController.getX(Hand.kLeft);
    // double ySpeed = -driveController.getY(Hand.kLeft);
    // double rot = (driveController.getTriggerAxis(Hand.kLeft)-driveController.getTriggerAxis(Hand.kRight));
    double xSpeed = this.xSpeed.getDouble(0);
    double ySpeed = this.ySpeed.getDouble(0);
    double rot = this.rot.getDouble(0);
    double angle = this.angle.getDouble(0);

    if (Math.abs(xSpeed) > 0.5) {
      xSpeed = 0.5 * getSign(xSpeed);
    }
    if (Math.abs(ySpeed) > 0.5) {
      ySpeed = 0.5 * getSign(ySpeed);
    }
    if (Math.abs(rot) > 0.5) {
      rot = 0.5 * getSign(rot);
    }

    if (zeroEntry.getBoolean(false)) {
      frontRightModule.setAngle(0);
      rearLeftModule.setAngle(0);
    }
    else drive(xSpeed, ySpeed, rot, true);
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //calculates speed and angle of modules and sets states
    // frontLeftModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    frontRightModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    rearLeftModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());
    // rearRightModule.setModuleState(xSpeed, ySpeed, rot, gyro.getAngle());

    // frontLeftStateEntry.setDoubleArray(new double [] {frontLeftModule.getSetVelocity(), frontLeftModule.getSetAngle()});
    frontRightStateEntry.setDouble(frontRightModule.getCurrentAngle());
    backLeftStateEntry.setDouble(rearLeftModule.getCurrentAngle());
    // backRightStateEntry.setDoubleArray(new double [] {rearRightModule.getSetVelocity(), rearRightModule.getSetAngle()});
  }

  public boolean getReset() {
    return resetEncoders.getBoolean(false);
  }

  public SwerveModule[] getModules() {
    return new SwerveModule[] {frontRightModule, rearLeftModule};
  }

  public boolean getZero() {
    return zeroEntry.getBoolean(false);
  }

  //returns +1 or -1 based on num's sign
  private double getSign(double num) {
    double sign = num/Math.abs(num);
    if (Double.isNaN(sign)) sign = 1;

    return sign;
}
}
