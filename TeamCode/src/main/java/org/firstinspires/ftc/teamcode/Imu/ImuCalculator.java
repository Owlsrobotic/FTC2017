package org.firstinspires.ftc.teamcode.Imu;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.LinkedList;

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

    BNO055IMU imu;
    LinearOpMode context;

    public ImuCalculator(BNO055IMU imu, LinearOpMode context){
        this.imu = imu;
        this.context = context;
    }

    @Override
    public void run() {

        prevTimeAccel = System.currentTimeMillis();
        prevTimeGyro = System.currentTimeMillis();

        int samples = 70;
        double accelThreshold = 1.0;
        double gyroThreshold = .05;
        LinkedList<Orientation> sampleWindow = new LinkedList<>();

        while(true){
            // TODO: 9/11/2017 : WRITE SENSOR UPDATE CODE

            double xAccel = imu.getLinearAcceleration().xAccel * -1;
            double yAccel = imu.getLinearAcceleration().yAccel * -1;
            double zGyro = imu.getAngularVelocity().xRotationRate;

            if (sampleWindow.size() <= samples) {
                sampleWindow.add(new Orientation(xAccel, yAccel, zGyro));
            } else {
                //Filtering
                sampleWindow.removeFirst();

                //Mechanical Filtering
                if (xAccel < accelThreshold && xAccel > -1 * accelThreshold) {
                    xAccel = 0;
                }
                if (yAccel < accelThreshold && yAccel > -1 * accelThreshold) {
                    yAccel = 0;
                }
                if (zGyro < gyroThreshold && zGyro > -1 * gyroThreshold) {
                    zGyro = 0;
                }

                sampleWindow.add(new Orientation(xAccel, yAccel, zGyro));
                double avgXAccel = sumSamples(sampleWindow).getPosX() / (double) samples;
                double avgYAccel = sumSamples(sampleWindow).getPosY() / (double) samples;
                double avgZGyro = sumSamples(sampleWindow).getRotationZ() / (double) samples;
                //Detecting Resting Robot
                if (avgXAccel < accelThreshold && avgXAccel > -1 * accelThreshold) {
                    prevVeloX = 0;
                    avgXAccel = 0;
                }
                if (avgYAccel < accelThreshold && avgYAccel > -1 * accelThreshold) {
                    prevVeloY = 0;
                    avgYAccel = 0;
                }
                if (avgZGyro < gyroThreshold && avgZGyro > -1 * gyroThreshold) {
                    prevWZ = 0;
                    avgZGyro = 0;
                }


                updateAccel(round(avgXAccel, 2), round(avgYAccel, 2));
                updateGyro(avgZGyro);
            }


//            updateAccel(-1 * imu.getLinearAcceleration().xAccel, -1 * imu.getLinearAcceleration().yAccel);
//            updateAccel(0, -10);
        }

    }

    public Orientation sumSamples(LinkedList<Orientation> samples){
        double sumXAccel = 0;
        double sumYAccel = 0;
        double sumZGyro = 0;

        for (Orientation sample : samples) {
            sumXAccel += sample.getPosX();
            sumYAccel += sample.getPosY();
            sumZGyro += sample.getRotationZ();
        }

        return new Orientation(sumXAccel, sumYAccel, sumZGyro);
    }

    public double round(double number, int places) {
        BigDecimal bd = new BigDecimal(number);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    public double areaUnderGraph(double x0, double y0, double xF, double yF){
        return (xF - x0)*((yF + y0) / 2);
    }

    //Give Acceleration Update in m/s/s
    public synchronized void updateAccel(double accelX, double accelY){
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

    public synchronized void updateGyro(double wZ){
        long currentTime = System.currentTimeMillis();

        double prevTimeSec = prevTimeGyro / 1000.0;
        double currentTimeSec = prevTimeGyro / 1000.0;

        totalWAreaZ += areaUnderGraph(prevTimeSec, prevWZ, currentTimeSec, wZ);

        prevTimeGyro = currentTime;
        prevWZ = wZ;

        currentRotationZ = totalWAreaZ;
    }

    public synchronized Orientation getOrientation(){
        return new Orientation(currentX, currentY, currentRotationZ);
    }

}
