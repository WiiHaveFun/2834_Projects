// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Brush;

public class ControlBrush extends CommandBase {
  Brush m_brush;
  double power;

  /** Creates a new ControlBrush. */
  public ControlBrush(Brush brush, double power) {
    m_brush = brush;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_brush);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_brush.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
