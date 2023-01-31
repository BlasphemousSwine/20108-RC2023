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
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@TeleOp
public class HurricaneTeleOpLift extends LinearOpMode
{

    double target = 0;
    double error;

    @Override
    public void runOpMode() {
        DcMotor lift1 = hardwareMap.dcMotor.get("lift1");
        DcMotor lift2 = hardwareMap.dcMotor.get("lift2");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper){
                target = -1800;
            }
            else if (gamepad1.left_bumper){
                target = -100;
            }

            error = target - lift1.getCurrentPosition();
            if (Math.abs(error) >= 70){
                lift1.setPower(-(error/180000));
                lift2.setPower(-(error/180000));
            }
            else{
                lift1.setPower(0);
                lift2.setPower(0);
            }

            telemetry.addLine(String.valueOf(lift1.getCurrentPosition()));
            telemetry.addLine(String.valueOf(lift2.getCurrentPosition()));
            telemetry.addLine(String.valueOf(error));
            telemetry.addLine(String.valueOf(-(error/180000)));
            telemetry.update();
        }
    }


}