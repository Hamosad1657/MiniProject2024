package frc.robot

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.*
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

object RobotContainer {
	const val JOYSTICK_DEADBAND = 0.1

	private val controllerA = CommandPS4Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)

	init {
		configureBindings()
		setDefaultCommands()
		SwerveSubsystem.heading
	}

	private fun configureBindings() {
//		// Toggle intake on circle button
//		commandControllerB.circle()
//			.toggleOnTrue(IntakeSubsystem.collectCommand().finallyDo { IntakeSubsystem.stopIntakeCommand() })
//
//		// Toggle aim shoot and load on cross button
//		commandControllerB.cross().toggleOnTrue(aimAndLoadWhenAimedCommand { SwerveSubsystem.pose })

//		// Aim shoot and load WITHOUT TURRET on square button
//		commandControllerB.square().onTrue(aimShooterAndHoodCommand { SwerveSubsystem.pose }
//			.alongWith(ConveyorSubsystem.loadWhenShooterAndHoodReady()))
//
//		// Turn turret to search for tags on options button
//		commandControllerB.options().onTrue(TurretSubsystem.searchForAnyTagCommand())

		controllerB.triangle().onTrue(TurretSubsystem.getToAngleCommand(90.degrees))

		controllerB.share().onTrue(InstantCommand({ TurretSubsystem.zeroEncoderAngle() }))

		controllerB.options().onTrue(InstantCommand({ HoodSubsystem.zeroHood() }))

		controllerB.cross()
			.toggleOnTrue(ShooterSubsystem.run { ShooterSubsystem.getToVelocity(AngularVelocity.fromRpm(0.0)) })

		controllerB.povUp().toggleOnTrue(LoaderSubsystem.runOpenLoop())
	}

	private fun setDefaultCommands() {
		// Temporarily commented out for testing
		// TODO: Change turret default command back to aimTurretCommand when done testing
		// TurretSubsystem.defaultCommand = TurretSubsystem.aimTurretCommand { SwerveSubsystem.pose }
//		TurretSubsystem.defaultCommand = TurretSubsystem.trackTargetCommand(Vision::bestTag)

		TurretSubsystem.defaultCommand =
			TurretSubsystem.openLoopTeleopCommand({ controllerB.r2Axis * 0.05 }, { controllerB.l2Axis * 0.05 })

		HoodSubsystem.defaultCommand = HoodSubsystem.teleopCommand { controllerB.leftY }

		SwerveSubsystem.defaultCommand = TeleopDriveCommand(
			SwerveSubsystem,
			vX = { -controllerA.leftY },
			vY = { -controllerA.leftX },
			omega = { controllerA.rightX },
			isFieldRelative = { true },
			isOpenLoop = false,
			headingCorrection = false,
		)
	}

	fun getAutonomousCommand(): Command? {
		return null
	}
}