package org.firstinspires.ftc.teamcode.Imu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by robotics on 9/11/2017.
 */

public class ImuCalculator extends Thread{

    //Accelerometer Variables
    double totalAccelAreaX = 0.0;
    double totalAccelAreaY = 0.0;

    double totalVeloAreaX = 0.0;
    double totalVeloAreaY = 0.0;

    double prevAccelX = 0.0;
    double prevAccelY = 0.0;

    double prevVeloX = 0.0;
    double prevVeloY = 0.0;

    public double currentX = 0.0;
    public double currentY = 0.0;

    long prevTimeAccel = 0;

    //Gyro Variables
    double totalWAreaZ = 0.0;
    double prevWZ = 0.0;
    long prevTimeGyro = 0;
    double currentRotationZ = 0.0;

    LinearOpMode context;

    public ImuCalculator(LinearOpMode context){
        this.context = context;
    }

    @Override
    public void run() {
        // TODO: 9/11/2017 : WRITE SENSOR UPDATE CODE
    }

    public double areaUnderGraph(double x0, double y0, double xF, double yF){
        return (xF - x0)*((yF + y0) / 2);
    }

    //Give Acceleration Update in m/s/s
    public void updateAccel(double accelX, double accelY){
        long currentTime = System.currentTimeMillis();

        double prevTimeSec = prevTimeAccel / 1000;
        double currentTimeSec = currentTime / 1000;

        totalAccelAreaX += areaUnderGraph(prevTimeSec, currentTimeSec, prevAccelX, accelX);
        totalAccelAreaY += areaUnderGraph(prevTimeSec, currentTimeSec, prevAccelY, accelY);

        totalVeloAreaX += areaUnderGraph(prevTimeSec, currentTimeSec, prevVeloX, totalAccelAreaX);
        totalVeloAreaY += areaUnderGraph(prevTimeSec, currentTimeSec, prevVeloY, totalAccelAreaY);

        prevAccelX = accelX;
        prevAccelY = accelY;
        prevVeloX = totalAccelAreaX;
        prevVeloY = totalAccelAreaY;
        prevTimeAccel = currentTime;

        currentX = totalVeloAreaX;
        currentY = totalVeloAreaY;
    }

    public void updateGyro(double wZ){
        long currentTime = System.currentTimeMillis();

        double prevTimeSec = prevTimeGyro / 1000;
        double currentTimeSec = prevTimeGyro / 1000;

        totalWAreaZ += areaUnderGraph(prevTimeSec, currentTimeSec, prevWZ, wZ);

        prevTimeGyro = currentTime;
        prevWZ = wZ;

        currentRotationZ = totalWAreaZ;
    }

    public Orientation getOrientation(){
        return new Orientation(currentX, currentY, currentRotationZ);
    }

}
