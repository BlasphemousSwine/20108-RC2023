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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class HurricaneTeleOpLift1 extends LinearOpMode
{

    double target = 0;
    double targetPosition;
    boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        DcMotor lift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor lift2 = hardwareMap.dcMotor.get("lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper && gamepad1.right_bumper != lastRightBumper) {
                target -= 1;
            }
            if (gamepad1.left_bumper) {
                target = 0;
            }

            if (target == -1) {
                targetPosition = -600;
                if (lift1.getCurrentPosition() > targetPosition) {
                    lift1.setPower(0.05);
                    lift2.setPower(0.05);
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            } else if (target == -2) {
                targetPosition = -1200;
                if (lift1.getCurrentPosition() > targetPosition) {
                    lift1.setPower(0.05);
                    lift2.setPower(0.05);
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            } else if (target == -3) {
                targetPosition = -2050;
                if (lift1.getCurrentPosition() > targetPosition) {
                    lift1.setPower(0.05);
                    lift2.setPower(0.05);
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            } else if (target == 0) {
                if (lift1.getCurrentPosition() < target) {
                    lift1.setPower(-0.01);
                    lift2.setPower(-0.01);
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }

            lastRightBumper = gamepad1.right_bumper;

            telemetry.addLine(String.valueOf(lift1.getCurrentPosition()));
            telemetry.addLine(String.valueOf(lift2.getCurrentPosition()));
            telemetry.addLine(String.valueOf(target));
            telemetry.update();
        }

    }
}