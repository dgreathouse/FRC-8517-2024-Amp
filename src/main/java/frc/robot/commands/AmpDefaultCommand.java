// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AmpSubsystem;

public class AmpDefaultCommand extends Command {
  AmpSubsystem m_amp;
  /** Creates a new AmpDefaultCommand. */
  public AmpDefaultCommand(AmpSubsystem _subsystem) {
    m_amp = _subsystem;
    addRequirements(_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.s_operatorController.R1().getAsBoolean()){
      m_amp.spin(0.2);
    }
    if(RobotContainer.s_operatorController.circle().getAsBoolean()){
      m_amp.rotate(90);
    }else {
      m_amp.rotate(0);
    }
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
