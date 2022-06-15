/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 0,false, false,0,false);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer,0,false,false,0,false);
  private final SwerveModule backLeft = new SwerveModule(Constants.rearLeftDrive, Constants.rearLeftSteer,0,false,false,0,false);
  private final SwerveModule backRight = new SwerveModule(Constants.rearRightDrive, Constants.rearRightSteer,0,false,false,0,false); 
  private final double trackWidth = Units.inchesToMeters(Constants.wheelBaseX);
  private final double trackLength = Units.inchesToMeters(Constants.wheelBaseY);
  private final Joystick transJoystick;
  private final Joystick rotJoystick;
  private final SwerveDriveKinematics m_kinematics;
  AHRS navx = new AHRS(Port.kMXP);

  public SwerveSubsystem() {
    transJoystick = new Joystick(Constants.transJoystickPort);
    rotJoystick = new Joystick(Constants.rotJoystickPort);
    m_kinematics = new SwerveDriveKinematics(new Translation2d(trackWidth,-trackLength),
    new Translation2d(trackWidth,trackLength),new Translation2d(-trackWidth,trackLength),
    new Translation2d(-trackWidth,-trackLength));
  }

  @Override
  public void periodic() {
    double x= transJoystick.getX();
    double y = transJoystick.getY();
    double rot = rotJoystick.getX();

    //Deadzones
    if (Math.abs(x)<Constants.deadzone){
      x = 0;
    }
    if (Math.abs(y)<Constants.deadzone){
      y = 0;
    }
    if(Math.abs(rot)<Constants.deadzone){
      rot = 0;
    }

    //input scaling
    x= x*Constants.maxSpeed;
    y= y*Constants.maxSpeed;
    rot = rot*Constants.maxSpeed;



  }
  
  public void resetGyro(){
    navx.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return new Rotation2d(getHeading());
  }
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
  }

}
