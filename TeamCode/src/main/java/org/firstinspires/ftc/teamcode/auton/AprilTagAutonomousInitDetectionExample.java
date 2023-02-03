/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Drivetrain1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain1;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Date;

@Autonomous (name = "AprilTagAutonomousInitDetectionExample", preselectTeleOp = "HurricaneTeleOp")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    DcMotorEx lift1;
    DcMotorEx lift2;
    AprilTagDetection tagOfInterest = null;
    SampleMecanumDrive drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new SampleMecanumDrive(hardwareMap);
        Servo leftlift = hardwareMap.servo.get("1");
        Servo rightlift = hardwareMap.servo.get("2");
        Servo leftclaw = hardwareMap.servo.get("4");
        Servo rightclaw = hardwareMap.servo.get("5");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftlift.setPosition(1);
        rightlift.setPosition(0.2);
        leftclaw.setPosition(1);
        rightclaw.setPosition(0);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        TrajectorySequence score = drivetrain.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))
                .forward(30)
                .addDisplacementMarker(() -> {
                    lift1.setTargetPosition(-1800);
                    lift2.setTargetPosition(-1800);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftlift.setPosition(0.9);
                    rightlift.setPosition(0.3);
                })
                //Junction
                .splineTo(new Vector2d(-30, 13), Math.toRadians(325))
                .addDisplacementMarker(() -> {
                    leftclaw.setPosition(0);
                    rightclaw.setPosition(1);
                    lift1.setTargetPosition(0);
                    lift2.setTargetPosition(0);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftlift.setPosition(0.7);
                    rightlift.setPosition(0.5);
                })
                .setReversed(true)
                //square2
                .splineTo(new Vector2d(-35, 40), Math.toRadians(90))
                .setReversed(false)
                //Pickup
                .splineTo(new Vector2d(-40, 20), Math.toRadians(200))
                .forward(14)
                .addDisplacementMarker(() -> {
                    leftclaw.setPosition(1);
                    rightclaw.setPosition(0);
                })
                .setReversed(true)
                .back(12)
                //square2
                .splineTo(new Vector2d(-38, 40), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    lift1.setTargetPosition(-1800);
                    lift2.setTargetPosition(-1800);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftlift.setPosition(0.9);
                    rightlift.setPosition(0.3);
                })
                .setReversed(false)
                //Junction
                .splineTo(new Vector2d(-30.5, 15.5), Math.toRadians(305))
                .addDisplacementMarker(() -> {
                    leftclaw.setPosition(0);
                    rightclaw.setPosition(1);
                })
                .setReversed(true)
                //square2
                .addDisplacementMarker(() -> {
                    lift1.setTargetPosition(0);
                    lift2.setTargetPosition(0);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftlift.setPosition(0.3);
                    rightlift.setPosition(0.9);
                })
                .splineTo(new Vector2d(-37, 40), Math.toRadians(90))
                .setReversed(false)
                //Pickup
                .splineTo(new Vector2d(-40, 20), Math.toRadians(190))
                .forward(12)
                .addDisplacementMarker(() -> {
                    leftclaw.setPosition(1);
                    rightclaw.setPosition(0);
                })
                .setReversed(true)
                .back(10)
                //square2
                .splineTo(new Vector2d(-35, 37), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    lift1.setTargetPosition(-1800);
                    lift2.setTargetPosition(-1800);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftlift.setPosition(0.9);
                    rightlift.setPosition(0.3);
                })
                .setReversed(false)
                //Junction
                .splineTo(new Vector2d(-30.5, 15.5), Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    leftclaw.setPosition(0);
                    rightclaw.setPosition(1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-32.5, 17.5), Math.toRadians(165))
                .build();
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested())

        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine(String.format("\nLift1", lift1.getCurrentPosition()));
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine(String.format("\nLift1", lift1.getCurrentPosition()));
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {   telemetry.addLine(String.format("\nLift1", lift1.getCurrentPosition()));
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        /* Actually do something useful */

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drivetrain.followTrajectorySequence(score);
            Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .forward(22)
                    .build();
            drivetrain.followTrajectory(goForward);
        }else if (tagOfInterest.id == MIDDLE){
            drivetrain.followTrajectorySequence(score);
        }else {
            drivetrain.followTrajectorySequence(score);
            Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                    .back(24)
                    .build();
            drivetrain.followTrajectory(goBackward);
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("\nLift1", lift1.getCurrentPosition()));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
    }
}