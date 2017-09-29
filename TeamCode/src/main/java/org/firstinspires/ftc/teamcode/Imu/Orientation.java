package com.ethanthemaster.Imu;

/**
 * Created by robotics on 9/11/2017.
 */

public class Orientation {

    double posX;
    double posY;
    double rotationZ;

    public Orientation(double posX, double posY, double rotationZ) {
        this.posX = posX;
        this.posY = posY;
        this.rotationZ = rotationZ;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public double getRotationZ() {
        return rotationZ;
    }
}
