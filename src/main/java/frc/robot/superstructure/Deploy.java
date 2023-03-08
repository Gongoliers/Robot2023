// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import com.thegongoliers.commands.output.DeployAndOperateCommand;
import com.thegongoliers.commands.output.OperateWithLockCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.ArmState;

public class Deploy extends SequentialCommandGroup {

  public Deploy(Arm arm, Claw claw, ArmState state) {
    addRequirements(arm, claw);
    addCommands(
        new InstantCommand(() -> arm.setExtendedState(state)),
        // TODO Am I using dependency injection correctly?
        new OperateWithLockCommand(
            arm,
            arm,
            new DeployAndOperateCommand(
                arm,
                arm,
                arm,
                // new InstantCommand(claw::close)
                new PrintCommand("done!")),
            1));
  }
}
