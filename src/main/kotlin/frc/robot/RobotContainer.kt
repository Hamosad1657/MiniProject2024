package frc.robot

import com.hamosad1657.lib.math.simpleDeadband
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.turret.TurretSubsystem
import frc.robot.subsystems.vision.Vision

object RobotContainer {
	const val JOYSTICK_DEADBAND = 0.1

	private val commandControllerA = CommandPS4Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val commandControllerB = CommandPS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val controllerB = PS4Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)

	private val turretTeleopTrigger = Trigger { controllerB.r3Button }

	init {
		ShooterSubsystem
		configureBindings()
		setDefaultCommands()
	}

	private fun configureBindings() {
		// Toggle intake on circle button
		commandControllerB.circle()
			.toggleOnTrue(IntakeSubsystem.collectCommand().finallyDo { IntakeSubsystem.stopIntakeCommand() })

		// Toggle aim shoot and load on cross button
		commandControllerB.cross().toggleOnTrue(aimAndLoadWhenAimedCommand { SwerveSubsystem.pose })

		// Toggle turret teleop control on right joystick press. Default command runs otherwise
		turretTeleopTrigger.toggleOnTrue(
			TurretSubsystem.closedLoopTeleopCommand(
				{ simpleDeadband(commandControllerB.rightX, JOYSTICK_DEADBAND) },
				{ simpleDeadband(-commandControllerB.rightX, JOYSTICK_DEADBAND) })
		)

		commandControllerB.options().onTrue(TurretSubsystem.searchForAnyTagCommand { Vision.currentBestTag })

	}

	private fun setDefaultCommands() {
		TurretSubsystem.defaultCommand = TurretSubsystem.aimTurretCommand { SwerveSubsystem.pose }
	}

	fun getAutonomousCommand(): Command? {
		return null
	}
}