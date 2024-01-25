package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.Hardwaremap;
import org.firstinspires.ftc.teamcode.helpers.Prop;

import java.util.List;

@Autonomous(name = "Retrive from stack test - Pixel Side",group = "robot")
@Disabled //DO NOT FORGET TO UNCOMMENT THIS FOR USE
public class VisionTesting extends LinearOpMode {
    Hardwaremap robot;

    @Override
    public void runOpMode() {
        //Init Robot
        robot = new Hardwaremap(this);
        robot.init();
        robot.initVision();

        robot.setDropperPosition(Hardwaremap.DroperPosition.CLOSED);

        //Waiting for start
        waitForStart();
        robot.tweetyBird.engage();

        //TUrning Around
        robot.tweetyBird.straightLineTo(0,5,0);
        robot.tweetyBird.straightLineTo(0,5,180);
        robot.tweetyBird.straightLineTo(0,-3,185);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        breakpoint();

        //Pixel Position 3

        robot.tweetyBird.speedLimit(0.5);
        robot.tweetyBird.straightLineTo(0,24,100);
        robot.tweetyBird.straightLineTo(8,24,102);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.setDropperPosition(Hardwaremap.DroperPosition.SINGLE);
        sleep(500);
        robot.setDropperPosition(Hardwaremap.DroperPosition.CLOSED);

        robot.tweetyBird.straightLineTo(0,24,100);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        breakpoint();

        //Get from the stack
        robot.tweetyBird.speedLimit(0.8);
        robot.tweetyBird.straightLineTo(0,50,100);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.tweetyBird.speedLimit(1);
        robot.tweetyBird.straightLineTo(-19,50,100);

        robot.intake.setPower(0.75);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        sleep(4000);
        robot.intake.setPower(0);

        robot.tweetyBird.straightLineTo(0,50,100);

        sleep(2000);
        robot.intake.setPower(1);

        sleep(2000);
        robot.intake.setPower(0);

        robot.tweetyBird.straightLineTo(60,50,100);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.tweetyBird.straightLineTo(60,35,0);





        //GO HOME
        breakpoint();

        while (opModeIsActive());

        robot.tweetyBird.stop();
    }

    private void breakpoint() {
        while (!gamepad1.a && opModeIsActive());
    }
}

