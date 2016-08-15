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
    double diffDriveRadius = 19.5; //ish cm

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
        path.add(new Coord(100,0));
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

    //Stuff to do, switch over to only storing wheel positions. Add position() and heading() functions that
    //deduce central position and heading from the two wheel positions. Add universal update function.
    //Update wheel positions individually based off of heading (be sure to store this in a variable beforehand)
    //and their respective encoder ticks. Move to a proportional or PID controller for movement, and make
    //driving and heading adjustments one in the same.
    @Override
    public void loop() {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0))) {
                path.remove(0);
            }
        }
    }

    /*
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
    */

    //DRY is weeping at the sight of this ungodly mess...
    private void updateState(int deltaL, int deltaR) {
        telemetry.addData("deltaL", deltaL);
        telemetry.addData("deltaR", deltaR);
        Coord lPos = new Coord(position.X + (Math.cos(heading + (Math.PI/2)) * diffDriveRadius), position.Y + (Math.sin(heading + (Math.PI/2)) * diffDriveRadius));
        Coord rPos = new Coord(position.X + (Math.cos(heading - (Math.PI/2)) * diffDriveRadius), position.Y + (Math.sin(heading - (Math.PI/2)) * diffDriveRadius));
        double lDistance = (deltaL / 1120.0) * (wheelDiameter * Math.PI);
        Tuple deltaLPos = new Tuple(Math.cos(heading) * lDistance, Math.sin(heading) * lDistance);
        double rDistance = (deltaR / 1120.0) * (wheelDiameter * Math.PI);
        Tuple deltaRPos = new Tuple(Math.cos(heading) * rDistance, Math.sin(heading) * rDistance);
        lPos = new Coord(lPos.sum(deltaLPos));
        rPos = new Coord(rPos.sum(deltaRPos));
        telemetry.addData("Delta Position L:", "(" + deltaLPos.X + "," + deltaLPos.Y + ")");
        telemetry.addData("Delta Position R:", "(" + deltaRPos.X + "," + deltaRPos.Y + ")");
        telemetry.addData("Position L:", "(" + lPos.X + "," + lPos.Y + ")");
        telemetry.addData("Position R:", "(" + rPos.X + "," + rPos.Y + ")");
        position = new Coord(lPos.sum(rPos).scale(1/2));
        telemetry.addData("Position:", "(" + position.X + "," + position.Y + ")");
        Tuple wheelDiff = lPos.difference(rPos);
        double dirtyHeading = Math.atan2(wheelDiff.X,wheelDiff.Y);
        heading = (heading + (dirtyHeading > 0 ? dirtyHeading : dirtyHeading + 2 * Math.PI)) % (2 * Math.PI);
        telemetry.addData("Heading:",heading);
    }

    private double headingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;
        if (error > Math.PI) {
            return error - (2 * Math.PI);
        }
        if (error < -Math.PI) {
            return error + (2 * Math.PI);
        }
        return error;
    }

    private boolean moveTo(Coord coord) {
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();
        if (position.distanceTo(coord) > 0.5) {
            double hError = headingError(heading,position.headingTo(coord));
            if (Math.abs(hError) > 0.05) {
                if (Math.abs(hError) < 0.1) {
                    if (hError > 0) {
                        leftMotor.setPower(0.3);
                        rightMotor.setPower(0.4);
                    }
                    if (hError < 0) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(0.3);
                    }
                }
                else {
                    if (hError > 0) {
                        leftMotor.setPower(-0.4);
                        rightMotor.setPower(0.4);
                    }
                    if (hError < 0) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(-0.4);
                    }
                }
                updateState(leftEnc-lastEncL,rightEnc-lastEncR);
                lastEncL = leftEnc;
                lastEncR = rightEnc;
                return false;
            }
            else {
                leftMotor.setPower(0.4);
                rightMotor.setPower(0.4);
                updateState(leftEnc-lastEncL,rightEnc-lastEncR);
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
    public final double X;
    public final double Y;

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

    public Tuple scale(double scalar) {
        double newX = this.X * scalar;
        double newY = this.Y * scalar;
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
        double angle = Math.atan2(vector.Y,vector.X);
        return (angle > 0 ? angle : angle + 2 * Math.PI);
    }
}