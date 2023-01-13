// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight.Pipeline;
public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  Limelight lime;
  SwerveSubsystem swervee;
  Joysticks joys;
  public AutoAlign(Limelight cam, SwerveSubsystem swere, Joysticks jo) {
    this.lime = cam;
    this.swervee = swere;
    this.joys = jo;
    addRequirements(cam);
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.setPipeline(Pipeline.TAG);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Y = (double) lime.hasTarg(()->lime.getForwardDistance());
    double rot = (double) lime.hasTarg(()->lime.getXoffset());
    swervee.setMotors(0, Y, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.joys.getLimeAlign();
  }
}