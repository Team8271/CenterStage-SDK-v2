package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.Hardwaremap;

@Autonomous(name = "TweetyBird Tester",group = "6")
//@Disabled //DO NOT FORGET TO UNCOMMENT THIS FOR USE
public class TweetyTest extends LinearOpMode {
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

        robot.tweetyBird.straightLineTo(-10,39,180);

        while (opModeIsActive());

        robot.tweetyBird.stop();
    }
}
