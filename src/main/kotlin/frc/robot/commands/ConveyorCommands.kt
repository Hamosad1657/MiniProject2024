package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.conveyor.ConveyorSubsystem
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

fun ConveyorSubsystem.loadBallsWhenReadyToShootCommand(): Command {
	val readyToShootSupplier = {
		ShooterSubsystem.withinTolerance() &&
			HoodSubsystem.withinTolerance() &&
			TurretSubsystem.withinTolerance()
	}
	return loadWhenCondition(readyToShootSupplier)
}

fun ConveyorSubsystem.loadWhenShooterAndHoodReady(): Command {
	val readyToShootSupplier = {
		ShooterSubsystem.withinTolerance() &&
			HoodSubsystem.withinTolerance()
	}
	return loadWhenCondition(readyToShootSupplier)
}


fun ConveyorSubsystem.loadWhenCondition(condition: () -> Boolean): Command {
	return run {
		if (condition()) {
			runConveyor(ShooterSubsystem.ballsPerSecs)
			runLoader(ShooterSubsystem.ballsPerSecs)
		} else {
			stopLoader()
			stopConveyor()
		}
	}.finallyDo {
		stopLoader()
		stopConveyor()
	}
}


