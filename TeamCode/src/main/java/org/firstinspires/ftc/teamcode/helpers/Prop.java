package org.firstinspires.ftc.teamcode.helpers;

public class Prop {
    int storedX = 0;
    int storedY = 0;
    double storedArea = 0;

    public Prop(int x, int y, double area) {
        storedX = x;
        storedY = y;
        storedArea = area;
    }

    public int getX() {
        return storedX;
    }
    public int getY() {
        return storedY;
    }
    public double getArea() {
        return storedArea;
    }
}
