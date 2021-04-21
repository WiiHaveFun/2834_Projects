// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BrushConstants;
import frc.robot.subsystems.Brush;

public class ReverseBrush extends CommandBase {
  Brush m_brush;

  /** Creates a new ReverseBrush. */
  public ReverseBrush(Brush brush) {
    m_brush = brush;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_brush);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_brush.setPower(-BrushConstants.kNominalPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_brush.setPower(BrushConstants.kNominalPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
