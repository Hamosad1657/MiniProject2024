package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.conveyor.ConveyorAndLoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem

fun ConveyorAndLoaderSubsystem.loadCommand(ballsPerSecs: () -> Double): Command {
	return withName("load") {
		ConveyorAndLoaderSubsystem.run { runLoader(ballsPerSecs()) }
			.andThen(ConveyorAndLoaderSubsystem.runOnce { stopLoader() })
	}
}

fun ConveyorAndLoaderSubsystem.runConveyorCommand(ballsPerSecsSupplier: () -> Double): Command {
	return ConveyorAndLoaderSubsystem.run { runConveyor(ballsPerSecsSupplier()) } andThen InstantCommand({ stopConveyor() })
}

fun ConveyorAndLoaderSubsystem.synchronizeBallLoadingCommand(): Command {
	return (runConveyorCommand { ShooterSubsystem.ballsPerSecs })
		.alongWith(loadCommand { ShooterSubsystem.ballsPerSecs })
}

