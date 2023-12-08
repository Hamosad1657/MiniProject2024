package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.conveyor.ConveyorSubsystem

fun ConveyorSubsystem.loadBallsCommand(): Command =
	run {
		runConveyor(conveyorBallsPerSecs)
		runLoader(loaderBallsPerSec)
	} andThen runOnce {
		stopConveyor()
		stopLoader()
	}


