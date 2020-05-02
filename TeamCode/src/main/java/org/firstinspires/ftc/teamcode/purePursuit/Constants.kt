package org.firstinspires.ftc.teamcode.purePursuit

object constants {
    val m = 15;
    val g = 9.8;
    val mu = 0.6;
    val maxMotorAmps = 20;
    val motorVolts = 3.3;
     val trackwidth = 18.0;
    val wheelBase = 18.0;
    val maxMotorPower = maxMotorAmps * motorVolts
    val numOfWheels = 4 // put in to scale the power to distribute it over the wheels
    val WheelVelToPowerConst =(m*g*mu / numOfWheels) / maxMotorPower
    // formula is power = force * velocity
    // what we want to do is get a net power of zero, so the velocity of the body is at the desired vel
    // so the power supplied has to equal the power lost to friction
    // power lost equals the force (friction) times the current velocity
    // therefore if we can calculate the power lost, we can supply that much to the wheels to make the net power zero
    // the best part is that no matter how fast the body is moving, if we set the power to equal what would be lost
    // at the desired velocity, the body will automatically adjust to that desired

}