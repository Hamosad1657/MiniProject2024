package frc.robot.subsystems.intake

import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object IntakeSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Intake.MOTOR_ID)


	fun collectCommand(): Command {
		return RunCommand({ motor.set(IntakeConstants.MOTOR_OUTPUT) }, this)
	}

	fun stopIntakeCommand(): Command {
		return InstantCommand({ motor.set(0.0) }, this)
	}
}