package frc.robot.commands

import com.hamosad1657.lib.commands.alongWith
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.mechanisms.BOILER_LOCATION
import frc.robot.mechanisms.HoodShooterState
import frc.robot.subsystems.conveyor.ConveyorSubsystem
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

fun aimWithOdometryCommand(robotPositionSupplier: () -> Pose2d): Command =
	aimShooterAndHoodCommand(robotPositionSupplier) alongWith
		TurretSubsystem.aimTurretCommand(robotPositionSupplier)

fun aimShooterAndHoodCommand(robotPositionSupplier: () -> Pose2d): Command =
	getToStateCommand {
		val distanceToBoiler = robotPositionSupplier().translation.getDistance(BOILER_LOCATION)
		HoodShooterState.fromDistance(distanceToBoiler.meters)
	}

fun getToStateCommand(stateSupplier: () -> HoodShooterState): Command =
	ShooterSubsystem.shootBallsCommand {
		stateSupplier().angularVelocity
	} alongWith HoodSubsystem.getToAngleCommand {
		stateSupplier().hoodAngle
	} finallyDo {
		HoodSubsystem.stopCommand()
	}

fun aimAndLoadWhenAimedCommand(robotPositionSupplier: () -> Pose2d): Command =
	aimWithOdometryCommand(robotPositionSupplier) alongWith
		ConveyorSubsystem.loadWhenReadyToShootCommand()
