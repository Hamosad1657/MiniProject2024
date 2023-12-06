package frc.robot.commands

import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.conveyor.ConveyorSubsystem


fun ConveyorSubsystem.runConveyorCommand(ballsPerSecs: () -> Double): Command =
	withName("runConveyor") {
		run { runConveyor(ballsPerSecs()) }
	}