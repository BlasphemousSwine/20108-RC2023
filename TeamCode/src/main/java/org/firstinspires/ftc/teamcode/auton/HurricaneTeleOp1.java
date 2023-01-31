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

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Drivetrain1;

@TeleOp
public class HurricaneTeleOp1 extends LinearOpMode {
    @Override

    public void runOpMode() {
        double imuright = 0;
        double forwardright = 0;
        double forwardleft = 0;
        double backleft = 0;
        double backright = 0;
        double orientation = 1;
        Drivetrain1 drivetrain = new Drivetrain1(hardwareMap);
        drivetrain.initialize();
        double imuforward = drivetrain.imu.getAngularOrientation().firstAngle;
        double imuleft = drivetrain.imu.getAngularOrientation().firstAngle+90;
        double imuback = drivetrain.imu.getAngularOrientation().firstAngle+180;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.start) {
                imuforward = drivetrain.imu.getAngularOrientation().firstAngle;
                imuright = drivetrain.imu.getAngularOrientation().firstAngle - 90;
                imuleft = drivetrain.imu.getAngularOrientation().firstAngle + 90;
                imuback = drivetrain.imu.getAngularOrientation().firstAngle + 180;
                if (imuright < -180) {
                    imuright = 180 - (java.lang.Math.abs(imuright + 180));
                }
                if (imuleft > 180) {
                    imuleft = -180 + (imuleft - 180);
                }
                if (imuback > 180) {
                    imuback = -180 + (imuback - 180);
                }
            }
            if (drivetrain.imu.getAngularOrientation().firstAngle > imuforward - 45 && drivetrain.imu.getAngularOrientation().firstAngle < imuforward + 45) {
                orientation = 1;
            } else if (drivetrain.imu.getAngularOrientation().firstAngle > imuright - 45 && drivetrain.imu.getAngularOrientation().firstAngle < imuright + 45) {
                orientation = 2;
            } else if (drivetrain.imu.getAngularOrientation().firstAngle > imuback - 45 && drivetrain.imu.getAngularOrientation().firstAngle < imuback + 45) {
                orientation = 3;
            } else if (drivetrain.imu.getAngularOrientation().firstAngle > imuleft - 45 && drivetrain.imu.getAngularOrientation().firstAngle < imuleft + 45) {
                orientation = 4;
            }
            if (orientation == 1){
                drivetrain.UpAndDown(
                        gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
            }
            if (orientation == 2){
                drivetrain.UpAndDown(
                        -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
            }
            if (orientation == 3){
                drivetrain.UpAndDown(
                        -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
            }
            if (orientation == 4){
                drivetrain.UpAndDown(
                        gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x),
                        -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y),
                        gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));
            }

            if(gamepad1.a) {
                drivetrain.initmotors();

            }
            telemetry.addLine(String.valueOf(drivetrain.imu.getAngularOrientation()));
            telemetry.addLine(String.valueOf(drivetrain.imu.getAngularOrientation()));
            telemetry.addLine(String.valueOf(drivetrain.imu.getAngularOrientation()));
            telemetry.addLine(String.valueOf(imuright));
            telemetry.addLine(String.valueOf(imuleft));
            telemetry.addLine(String.valueOf(imuback));
            telemetry.addLine(String.valueOf(orientation));
            telemetry.addLine(String.format("fr", drivetrain.fr.getCurrentPosition()));
            telemetry.addLine(String.format("br", drivetrain.br.getCurrentPosition()));
            telemetry.addLine(String.format("fl", drivetrain.fl.getCurrentPosition()));
            telemetry.addLine(String.format("bl", drivetrain.bl.getCurrentPosition()));
            telemetry.update();

        }
    }
}