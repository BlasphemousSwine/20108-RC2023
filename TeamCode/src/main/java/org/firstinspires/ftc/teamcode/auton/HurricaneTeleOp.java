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
import org.firstinspires.ftc.teamcode.Drivetrain;

@TeleOp
public class HurricaneTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setVelocity[0] = -gamepad1.left_stick_x;
            drivetrain.setVelocity[1] = -gamepad1.left_stick_y;
            drivetrain.setVelocity[2] = gamepad1.right_stick_x;
            drivetrain.update();
            telemetry.addLine(String.valueOf(drivetrain.position[0]));
            telemetry.addLine(String.valueOf(drivetrain.position[1]));
            telemetry.addLine(String.valueOf(drivetrain.position[2]));
            telemetry.update();
            if(gamepad1.start) {
                drivetrain.initializeIMU();
            }


        }
    }
}