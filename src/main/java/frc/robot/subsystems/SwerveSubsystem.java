/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  //init swerve module calculation objects
  private final Translation2d frontLeftLocation = new Translation2d(Constants.swerveModuleDistance, Constants.swerveModuleDistance);
  private final Translation2d frontRightLocation = new Translation2d(Constants.swerveModuleDistance, -Constants.swerveModuleDistance);
  private final Translation2d backLeftLocation = new Translation2d(-Constants.swerveModuleDistance, Constants.swerveModuleDistance);
  private final Translation2d backRightLocation = new Translation2d(-Constants.swerveModuleDistance, -Constants.swerveModuleDistance);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  //init swerve drive objects
  private final SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftSteer, Constants.frontLeftDrive);
  private final SwerveModule frontRightModule = new SwerveModule(Constants.frontRightSteer, Constants.frontRightDrive);
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.rearLeftSteer, Constants.rearLeftDrive);
  private final SwerveModule rearRightModule = new SwerveModule(Constants.rearRightSteer, Constants.rearRightDrive);

  //init gyro
  private final AnalogGyro gyro = new AnalogGyro(0);

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
    double xSpeed = -driveController.getY(Hand.kLeft) * Constants.maxSpeed;
    double ySpeed = driveController.getX(Hand.kLeft) * Constants.maxSpeed;
    double rot = (driveController.getTriggerAxis(Hand.kLeft)-driveController.getTriggerAxis(Hand.kRight)) * Constants.maxAngularSpeed;

    drive(xSpeed, ySpeed, rot, true);
  }
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //saves current speeds of robot
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot * (180/Math.PI), Rotation2d.fromDegrees(gyro.getAngle()));
    
    //produces array of each individual serve module state
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    frontLeftStateEntry.setDoubleArray(new double [] {moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getDegrees()});
    frontRightStateEntry.setDoubleArray(new double [] {moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getDegrees()});
    backLeftStateEntry.setDoubleArray(new double [] {moduleStates[2].speedMetersPerSecond, moduleStates[0].angle.getDegrees()});
    backRightStateEntry.setDoubleArray(new double [] {moduleStates[3].speedMetersPerSecond, moduleStates[1].angle.getDegrees()});

    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    rearLeftModule.setDesiredState(moduleStates[2]);
    rearRightModule.setDesiredState(moduleStates[3]);
  }
}
