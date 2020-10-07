package org.firstinspires.ftc.teamcode.StaticSparky

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous(name = "SparkyTests", group = "StaticDischarge")
class SparkyRedAutoLeft : SparkAutoBase() {
    override fun runOpMode() {
        // UNCOMMENT THIS IF SOUNDS ARE NEEDED
        val robot = TestRobot(hardwareMap, telemetry)
        waitForStart()
        robot.move(-0.5 * TILE_LENGTH, 2 * TILE_LENGTH)
        robot.move(0.5 * TILE_LENGTH, 2 * TILE_LENGTH)
        robot.move(1.5 * TILE_LENGTH, -3 * TILE_LENGTH)
        robot.turnTo(180.0)
        robot.move(0.0, TILE_LENGTH)

        // first build a sequential list of commands
        // then, program out your instructions one by one
    }
}