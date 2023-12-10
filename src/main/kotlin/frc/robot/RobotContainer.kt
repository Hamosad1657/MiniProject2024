package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.collectCommand
import frc.robot.commands.stopIntakeCommand
import frc.robot.subsystems.intake.IntakeSubsystem

object RobotContainer {
	private val controllerA = CommandPS4Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)

	init {
		configureBindings()
		setDefaultCommands()
	}

	private fun configureBindings() {}

	private fun setDefaultCommands() {
		// Toggle intake on circle button
		controllerB.circle()
			.toggleOnTrue(IntakeSubsystem.collectCommand().finallyDo { IntakeSubsystem.stopIntakeCommand() })
	}

	fun getAutonomousCommand(): Command? {
		return null
	}
}