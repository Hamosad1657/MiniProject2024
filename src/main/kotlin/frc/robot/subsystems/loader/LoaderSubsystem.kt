package frc.robot.subsystems.loader

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object LoaderSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Loader.MOTOR_ID)

	fun LoadCommand(ballsPerSecs: () -> Double): Command {
		return RunCommand({
			val setpoint = LoaderConstants.CONVEYOR_VELOCITY_CONVERSION * ballsPerSecs()
			motor.set(ControlMode.Velocity, setpoint)
		})
	}
}