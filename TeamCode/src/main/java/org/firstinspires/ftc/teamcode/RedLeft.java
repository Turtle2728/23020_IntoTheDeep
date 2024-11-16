package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RedLeft", group = "RedAuto", preselectTeleOp = "KLA2024")
public class RedLeft extends LinearOpMode {

    DcMotor armMotor;
    Servo wrist;
    Servo gripper;

    int start_delay = 0;

    public void armadjust(double armPower, int armTarget, double wristTarget) {

        armMotor.setTargetPosition(armTarget);
        wrist.setPosition(wristTarget);
        armMotor.setPower(armPower);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void gripper1(double gripperTarget) {
        gripper.setPosition(gripperTarget);
    }

    public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-16.5,-65.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        armMotor = hardwareMap.dcMotor.get("ARM");

        gripper = hardwareMap.servo.get("gripper1");

        wrist = hardwareMap.servo.get("wrist");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.

           /* if (currentGamepad2.x && !previousGamepad2.x) {
                start_delay = start_delay + 500;
                telemetry.addData("delay", start_delay);
                telemetry.update();
            } */    //시작 딜레이, 한번 누를때마다 0.5초



        }

        //customSleep(start_delay);  //시작 딜레이

        // Share the CPU.
        sleep(20);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .stopAndAdd(()->armadjust(1,1400,0.3))
                .stopAndAdd(()->gripper1(0.9))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-7.5,-39.5),Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(()->armadjust(1,900,0.3))
                .waitSeconds(1)
                .stopAndAdd(()->gripper1(0.3))
                .waitSeconds(1)
                .lineToY(-48)
                .waitSeconds(1);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-7.5,-48,Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-47,-48))
                .stopAndAdd(()->armadjust(1,400,0.15))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-47,-37))
                .waitSeconds(1)
                .stopAndAdd(()->armadjust(1,250,0.15))
                .stopAndAdd(()->gripper1(0.9))
                .waitSeconds(1)
                .stopAndAdd(()->armadjust(1,2100,0.65))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-59,-54))
                .waitSeconds(1)
                .turn(Math.toRadians(-45))
                .stopAndAdd(()->gripper1(0.6))
                .waitSeconds(2)
                .turn(Math.toRadians(45));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-59,-54,Math.toRadians(90)))
                .stopAndAdd(()->armadjust(1,400,0.4))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-48,-12),Math.toRadians(90))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-20,-12))
                .stopAndAdd(()->armadjust(1,0,0.4));

        Action trajectoryActionCloseOut1 = tab1.fresh()
                .build();

        Action trajectoryActionCloseOut2 = tab2.fresh()
                .build();

        Action trajectoryActionCloseOut3 = tab3.fresh()
                .build();

        Action trajectoryAction1;
        trajectoryAction1 = tab1.build();

        Action trajectoryAction2;
        trajectoryAction2 = tab2.build();

        Action trajectoryAction3;
        trajectoryAction3 = tab3.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1,
                        trajectoryActionCloseOut1,
                        trajectoryAction2,
                        trajectoryActionCloseOut2,
                        trajectoryAction3,
                        trajectoryActionCloseOut3
                )
        );


    }



}