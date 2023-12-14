package frc.robot.commands

import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.mechanisms.BOILER_LOCATION
import frc.robot.mechanisms.HoodShooterState
import frc.robot.subsystems.conveyor.ConveyorSubsystem
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

fun aimWithOdometryCommand(robotPositionSupplier: () -> Pose2d): Command {
	return aimShooterAndHoodFromOdometryCommand(robotPositionSupplier)
		.alongWith(TurretSubsystem.aimTurretCommand(robotPositionSupplier))
}

fun aimShooterAndHoodFromOdometryCommand(robotPositionSupplier: () -> Pose2d): Command {
	return aimShooterAndHoodCommand {
		(robotPositionSupplier().translation.getDistance(BOILER_LOCATION)).meters
	}
}

fun aimShooterAndHoodCommand(distanceToBoiler: () -> Length): Command {
	return getToStateCommand { HoodShooterState.fromLength(distanceToBoiler()) }
}

fun getToStateCommand(stateSupplier: () -> HoodShooterState): Command {
	return ShooterSubsystem.shootBallsCommand(stateSupplier().angularVelocity)
		.alongWith(HoodSubsystem.getToAngleCommand { stateSupplier().hoodAngle }).finallyDo {
			ShooterSubsystem.stopShooter()
			HoodSubsystem.stopCommand()
		}
}

fun aimAndLoadWhenAimedCommand(robotPositionSupplier: () -> Pose2d): Command {
	return aimWithOdometryCommand(robotPositionSupplier)
		.alongWith(ConveyorSubsystem.loadBallsWhenReadyToShootCommand())
}