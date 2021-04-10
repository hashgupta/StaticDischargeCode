package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer


//Not meant for real use
class MockedLocalizer : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set


    override fun update() {

    }
}