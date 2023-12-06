package frc.robot.subsystems.conveyor

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ConveyorSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Conveyor.MOTOR_ID).apply {
		config_kP(0, frc.robot.subsystems.conveyor.ConveyorConstants.PID_GAINS.kP)
		config_kI(0, ConveyorConstants.PID_GAINS.kI)
		config_kD(0, ConveyorConstants.PID_GAINS.kD)
	}

	fun runConveyor(ballsPerSecs: Double) {
		motor.set(ControlMode.Velocity, ballsPerSecs)
	}

	val ballsPerSecs: Double
		get() = motor.selectedSensorVelocity * ConveyorConstants.CONVEYOR_VELOCITY_RATIO
}