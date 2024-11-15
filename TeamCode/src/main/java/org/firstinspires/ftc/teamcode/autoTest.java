package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "autoTest", group = "Autonomous")
public class autoTest extends LinearOpMode {
    public class ARM {
        private DcMotor ARM;

        public ARM(HardwareMap hardwareMap) {
            ARM = hardwareMap.dcMotor.get("ARM");
            ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ARM.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ARMUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ARM.setPower(0.8);
                    initialized = true;
                }

                double pos = ARM.getCurrentPosition();
                packet.put("ARMPos", pos);
                if (pos < 1100.0) {
                    return true;
                } else {
                    ARM.setPower(0);
                    return false;
                }
            }
        }
        public Action ARMUp() {
            return new ARMUp();
        }

        public class ARMDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ARM.setPower(-0.8);
                    initialized = true;
                }

                double pos = ARM.getCurrentPosition();
                packet.put("ARMPos", pos);
                if (pos > 0) {
                    return true;
                } else {
                    ARM.setPower(0);
                    return false;
                }
            }
        }
        public Action ARMDown(){
            return new ARMUp();
        }
    }

    public class gripper1 {
        private Servo gripper1;

        public gripper1(HardwareMap hardwareMap) {
            gripper1 = hardwareMap.get(Servo.class, "gripper1");
        }

        public class Closegripper1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper1.setPosition(0.95);
                return false;
            }
        }
        public Action closegripper1() {
            return new Closegripper1();
        }

        public class Opengripper1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper1.setPosition(0.6);
                return false;
            }
        }
        public Action opengripper1() {
            return new Opengripper1();
        }
    }

    public class wrist {
        private Servo wrist;

        public wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        public class wristDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.15);
                return false;
            }
        }
        public Action wristDown() {
            return new wristDown();
        }

        public class wristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.55);
                return false;
            }
        }
        public Action wristUp() {
            return new wristUp();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-30, -65.8, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        gripper1 gripper1 = new gripper1(hardwareMap);
        ARM ARM = new ARM(hardwareMap);
        wrist wrist = new wrist(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-6,-31),Math.toRadians(90))
                .waitSeconds(2);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-6,-31),Math.toRadians(90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-6,-31),Math.toRadians(90));
        Action trajectoryActionCloseOut = tab1.fresh()
                .lineToY(-40)
                .build();

        // actions that need to happen on init; for instance, a gripper1 tightening.
        Actions.runBlocking(gripper1.closegripper1());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        ARM.ARMUp(),
                        gripper1.closegripper1(),
                        wrist.wristUp(),
                        trajectoryActionChosen,
                        wrist.wristDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}