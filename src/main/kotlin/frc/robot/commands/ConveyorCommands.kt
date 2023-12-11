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
	return run {
		if (readyToShootSupplier()) {
			runConveyor(conveyorBallsPerSecs)
			runLoader(loaderBallsPerSec)
		} else {
			stopConveyor()
			stopLoader()
		}
	}.finallyDo {
		stopConveyor()
		stopLoader()
	}
}


