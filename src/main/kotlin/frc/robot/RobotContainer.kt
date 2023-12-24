package frc.robot

import com.hamosad1657.lib.units.degrees
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.getToAngleCommand
import frc.robot.commands.openLoopTeleopCommand
import frc.robot.subsystems.turret.TurretSubsystem

object RobotContainer {
	const val JOYSTICK_DEADBAND = 0.1

	private val commandControllerA = CommandPS4Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val commandControllerB = CommandPS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val controllerB = PS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)

	private val turretTeleopTrigger = Trigger { controllerB.r3Button }

	init {
		configureBindings()
		setDefaultCommands()
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

		commandControllerB.triangle().onTrue(TurretSubsystem.getToAngleCommand(90.degrees))

		commandControllerB.share().onTrue(InstantCommand({ TurretSubsystem.resetEncoderAngle() }))
	}

	private fun setDefaultCommands() {
		// Temporarily commented out for testing
		// TODO: Change turret default command back to aimTurretCommand when done testing
		// TurretSubsystem.defaultCommand = TurretSubsystem.aimTurretCommand { SwerveSubsystem.pose }
		TurretSubsystem.defaultCommand = TurretSubsystem.openLoopTeleopCommand(
			{ controllerB.l2Axis * 0.3 },
			{ controllerB.r2Axis * 0.3 })
	}

	fun getAutonomousCommand(): Command? {
		return TurretSubsystem.getToAngleCommand(90.degrees)
	}
}