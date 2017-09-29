package com.ethanthemaster.Imu;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by robotics on 9/11/2017.
 */

public class ImuCalculator implements Runnable{

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

    long prevTimeAccel;

    //Gyro Variables
    double totalWAreaZ = 0.0;
    double prevWZ = 0.0;

    double currentRotationZ = 0.0;

    long prevTimeGyro;

    //LinearOpMode context;

    /*public ImuCalculator(LinearOpMode context){
        this.context = context;
    }*/

    @Override
    public void run() {

        prevTimeAccel = System.currentTimeMillis();
        prevTimeGyro = System.currentTimeMillis();

        while(true){
            // TODO: 9/11/2017 : WRITE SENSOR UPDATE CODE
            updateAccel(0, -10);
        }

    }

    public double areaUnderGraph(double x0, double y0, double xF, double yF){
        return (xF - x0)*((yF + y0) / 2);
    }

    //Give Acceleration Update in m/s/s
    public void updateAccel(double accelX, double accelY){
        long currentTime = System.currentTimeMillis();

        double prevTimeSec = prevTimeAccel / 1000.0;
        double currentTimeSec = currentTime / 1000.0;

        totalAccelAreaX += areaUnderGraph(prevTimeSec, prevAccelX, currentTimeSec, accelX);
        totalAccelAreaY += areaUnderGraph(prevTimeSec, prevAccelY, currentTimeSec, accelY);

        totalVeloAreaX += areaUnderGraph(prevTimeSec, prevVeloX, currentTimeSec, totalAccelAreaX);
        totalVeloAreaY += areaUnderGraph(prevTimeSec, prevVeloY, currentTimeSec, totalAccelAreaY);

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

        double prevTimeSec = prevTimeGyro / 1000.0;
        double currentTimeSec = prevTimeGyro / 1000.0;

        totalWAreaZ += areaUnderGraph(prevTimeSec, prevWZ, currentTimeSec, wZ);

        prevTimeGyro = currentTime;
        prevWZ = wZ;

        currentRotationZ = totalWAreaZ;
    }

    public Orientation getOrientation(){
        return new Orientation(currentX, currentY, currentRotationZ);
    }

}
