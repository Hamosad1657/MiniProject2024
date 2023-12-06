package com.hamosad1657.lib.commands

import edu.wpi.first.wpilibj2.command.*

fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)

infix fun Command.andThen(next: Command): Command = this.andThen(next)
infix fun Command.until(condition: () -> Boolean): Command = this.until(condition)
fun SubsystemBase.withName(commandName: String, commandSupplier: () -> Command): Command =
	commandSupplier().also { it.name = "$commandName : $name" }