package frc.robot.subsystems.conveyor

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ConveyorSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Conveyor.MOTOR_ID)

	init {
		motor.config_kP(0, ConveyorConstants.kP)
		motor.config_kI(0, ConveyorConstants.kI)
		motor.config_kD(0, ConveyorConstants.kD)
	}

	fun runConveyorCommand(ballsPerSecs: () -> Double): Command {

		val setpoint = ConveyorConstants.CONVEYOR_VELOCITY_CONVERSION * ballsPerSecs()

		return RunCommand({ motor.set(ControlMode.Velocity, setpoint) })
	}
}