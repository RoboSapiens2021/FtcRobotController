/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.FourWheelMacanumDrive;
import org.firstinspires.ftc.teamcode.util.Log;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "GamePadTeleOp", group = "TeleOp")
/**
 * POV Mode uses left stick to go forward, and right stick to turn
 */
public class GamePadTeleOp extends LinearOpMode {
    private static final Log LOG = Logger.getInstance();

    @Override
    public void runOpMode() {

        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        // run until the end of the match (driver presses STOP)
        FourWheelMacanumDrive driveTrain = robot.getDriveTrain();
        driveTrain.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            driveTrain.fieldDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            Robot.getInstance().getSpinner().spin((gamepad1.left_trigger - gamepad1.right_trigger) / 2);

            if (gamepad1.y) {
                Robot.getInstance().getImu().resetPosition();
            }
        }
    }


//    private double[] getMotorPower() {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double r = gamepad1.right_stick_x;
//
//        double max = Math.max(Math.abs(y), Math.max(Math.abs(x), Math.abs(r)));
//
//        double[] motorPower = new double[4];
//        motorPower[0] = (y + x + r) / max; // lf
//        motorPower[1] = (y - x + r) / max; // lr
//        motorPower[2] = (y + x - r) / max; // rr
//        motorPower[3] = (y - x - r) / max; // rf
//
//        return motorPower;
//    }

}



