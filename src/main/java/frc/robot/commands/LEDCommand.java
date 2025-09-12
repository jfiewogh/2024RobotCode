// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem.BlinkinMode;

public class LEDCommand extends Command {
  private BlinkinLedSubsystem ledSubsystem;
  private BlinkinMode currentMode = BlinkinMode.Off;

  public LEDCommand(BlinkinLedSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentMode) {
      case Off:
        currentMode = BlinkinMode.HoldingPiece;
        break;
      case HoldingPiece:
        currentMode = BlinkinMode.ShooterReady;
        break;
      case ShooterReady:
        currentMode = BlinkinMode.Source;
        break;
      case Source:
        currentMode = BlinkinMode.Amp;
        break;
      case Amp:
        currentMode = BlinkinMode.HoldingPiece;
        break;
      default:
        break;
    }
    ledSubsystem.setMode(currentMode);
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