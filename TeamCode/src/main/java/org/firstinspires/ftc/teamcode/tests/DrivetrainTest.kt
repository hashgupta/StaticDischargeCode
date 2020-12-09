package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Controllers.DriveTrain
import org.firstinspires.ftc.teamcode.hardware.general.Motor
import org.firstinspires.ftc.teamcode.localizers.MockedLocalizer
import org.firstinspires.ftc.teamcode.pipelines.RingPipeline
import org.firstinspires.ftc.teamcode.purePursuit.FastPurePursuit
import org.firstinspires.ftc.teamcode.staticSparky.Positions
import org.firstinspires.ftc.teamcode.staticSparky.SparkAutoBase
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import kotlin.math.PI

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DrivetrainTest {
    @Test
    fun testDrivetrain() {
        val pursuiter = FastPurePursuit(MockedLocalizer(), Pose2d(0.0,0.0,0.0))
        pursuiter.setStartPoint(Positions.startLeftRed)
        pursuiter.addRelativePoint(-0.5 * SparkAutoBase.TILE_LENGTH, 2 * SparkAutoBase.TILE_LENGTH, 0.0)

        val goalZone = Pose2d(Positions.aZoneRed, 0.0)

        pursuiter.addPoint(goalZone + Pose2d(-9.0, -9.0))
        pursuiter.addPoint(Positions.startRightRed + Pose2d(SparkAutoBase.TILE_LENGTH *0.5 + 9.0, Constants.trackwidth * 0.5, PI ))


        println(pursuiter.waypoints)
        println(pursuiter.waypoints[pursuiter.index+2])

    }
}