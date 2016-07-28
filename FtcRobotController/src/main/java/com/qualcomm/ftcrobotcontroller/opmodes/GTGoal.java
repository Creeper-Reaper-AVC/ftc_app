package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.util.ArrayList;

/*
TODO:
  * Combine update header and position functions and allow heading to be updated while driving. This
      allows the bot to make corrections while driving if one motor lags behind the other.
  * Clean up this mess.
*/

public class GTGoal extends OpMode {

    //Constants
    double wheelDiameter = 9.15; //cm
    double pivotRadius = 19.5; //ish cm

    //Declare robot variables:
    DcMotor leftMotor;
    DcMotor rightMotor;
    int lastEncL;
    int lastEncR;
    Coord position;
    double heading; //Radians

    ArrayList<Coord> path;

    @Override
    public void init() {
        path = new ArrayList<>();
        path.add(new Coord(230,0));
        path.add(new Coord(230,250));
        path.add(new Coord(350,250));
        leftMotor = hardwareMap.dcMotor.get("L");
        rightMotor = hardwareMap.dcMotor.get("R");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        lastEncL = leftMotor.getCurrentPosition();
        lastEncR = rightMotor.getCurrentPosition();
        position = new Coord(0,0);
        heading = 0;
    }

    @Override
    public void loop() {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0))) {
                path.remove(0);
            }
        }
    }

    public void updatePosition(int deltaL, int deltaR) {
        int deltaTicks = Math.round((deltaL + deltaR) / 2);
        double deltaRots = deltaTicks / 1120.0;
        double deltaDistance = deltaRots * (wheelDiameter * Math.PI);
        Tuple deltaPosition = new Tuple(Math.cos(heading) * deltaDistance, Math.sin(heading) * deltaDistance);
        position = new Coord(position.sum(deltaPosition));
    }

    public void updateHeading(int deltaL, int deltaR) {
        int deltaTicks = Math.round((Math.abs(deltaL) + Math.abs(deltaR)) / 2);
        double deltaRots = deltaTicks / 1120.0;
        double deltaDistance = deltaRots * (wheelDiameter * Math.PI);
        if (deltaL < deltaR) {
            heading = heading + (deltaDistance / pivotRadius);
        }
        if (deltaR < deltaL) {
            heading = heading - (deltaDistance / pivotRadius);
        }
    }

    private boolean moveTo(Coord coord) {
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();
        if (position.distanceTo(coord) > 0.5) {
            if (Math.abs(position.headingTo(coord) - heading) > 0.05) {
                if (Math.abs(position.headingTo(coord) - heading) < 0.1) {
                    if (position.headingTo(coord) > heading) {
                        leftMotor.setPower(0.3);
                        rightMotor.setPower(0.4);
                    }
                    if (position.headingTo(coord) < heading) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(0.3);
                    }
                }
                else {
                    if (position.headingTo(coord) > heading) {
                        leftMotor.setPower(-0.4);
                        rightMotor.setPower(0.4);
                    }
                    if (position.headingTo(coord) < heading) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(-0.4);
                    }
                }
                updateHeading(leftEnc-lastEncL,rightEnc-lastEncR);
                lastEncL = leftEnc;
                lastEncR = rightEnc;
                return false;
            }
            else {
                leftMotor.setPower(0.4);
                rightMotor.setPower(0.4);
                updatePosition(leftEnc-lastEncL,rightEnc-lastEncR);
                lastEncL = leftEnc;
                lastEncR = rightEnc;
                return false;
            }
        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }
    }
}

class Tuple {
    public double X;
    public double Y;

    public Tuple(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    public Tuple sum(Tuple tuple) {
        double newX = this.X + tuple.X;
        double newY = this.Y + tuple.Y;
        return new Tuple(newX,newY);
    }

    public Tuple difference(Tuple tuple) {
        double newX = this.X - tuple.X;
        double newY = this.Y - tuple.Y;
        return new Tuple(newX,newY);
    }
}

class Coord extends Tuple {
    public Coord(double X, double Y) {
        super(X,Y);
    }

    public Coord(Tuple t) {
        super(t.X,t.Y);
    }

    public double distanceTo(Coord coord) {
        Tuple vector = coord.difference(this);
        return Math.sqrt(Math.pow(vector.X,2) + Math.pow(vector.Y,2));
    }

    public double headingTo(Coord coord) {
        Tuple vector = coord.difference(this);
        return Math.atan2(vector.Y,vector.X);
    }
}