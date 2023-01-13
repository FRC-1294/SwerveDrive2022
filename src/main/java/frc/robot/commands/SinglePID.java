// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class SinglePID extends CommandBase {
  /** Creates a new SinglePID. */
  SwerveSubsystem swerve;
  SwerveModule selectedModule;
  Joysticks joys;

  public SinglePID(SwerveModule module) {
    this.selectedModule = module;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.kP = SmartDashboard.getNumber("kP", 0);
    Constants.kI = SmartDashboard.getNumber("kI", 0);
    Constants.kD = SmartDashboard.getNumber("kD", 0);

    Double sp = SmartDashboard.getNumber("Setpoint", 0);
    
    SmartDashboard.putNumber("setPointReal", Constants.tuningSetpoint);
    selectedModule.updatePositions(sp);
  
    SmartDashboard.putNumber("Module1CurrentROT",this.selectedModule.getRotPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
