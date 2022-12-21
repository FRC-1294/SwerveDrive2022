// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerveee;
  Joysticks joyee;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  public DefaultDriveCmd(Joysticks joys, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joyee = joys;
    this.swerveee = swerve;
    xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond*10);
    yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond*10);
    turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond*30);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("entered execute");
    double x= this.joyee.getX();
    double y = this.joyee.getY();
    double rot = this.joyee.getRot();
  
    x = Math.abs(x) > 0.15 ? x : 0.0;
    y = Math.abs(y) > 0.15 ? y : 0.0;
    rot = Math.abs(rot) > 0.05 ? rot : 0.0;
      
      // 3. Make the driving smoother
    x = xLimiter.calculate(x) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
    y = yLimiter.calculate(y) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
    rot= turningLimiter.calculate(rot)
          * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond*3;
    swerveee.setMotors(x, y, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveee.setMotors(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
