package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;


@TeleOp (name = "DriverMode_Test", group = "Driver")

public class DriverMode extends LinearOpMode
{

    //motoare
    private DcMotor mers_left = null;
    private DcMotor mers_right = null;
    private DcMotor glisare = null;
    private DcMotor ridicare_cutie = null;
    private DcMotor ridicare_perii = null;
    private DcMotor rotire_perii = null;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while (opModeIsActive())
        {
            firstGamepad();
            secondGamepad();
        }
    }

    private void initialise()
    {
        //mapare
        mers_left = hardwareMap.dcMotor.get("mers_left");
        mers_right = hardwareMap.dcMotor.get("mers_right");
        glisare = hardwareMap.dcMotor.get("glisare");
        ridicare_cutie = hardwareMap.dcMotor.get("ridicare_cutie");
        ridicare_perii = hardwareMap.dcMotor.get("ridicare_perii");
        rotire_perii = hardwareMap.dcMotor.get("rotire_perii");

        //setare puteri la 0
        mers_left.setPower(0);
        mers_right.setPower(0);
        glisare.setPower(0);
        ridicare_cutie.setPower(0);
        ridicare_perii.setPower(0);
        rotire_perii.setPower(0);

        //setare directii
        mers_left.setDirection(DcMotorSimple.Direction.REVERSE);
        mers_right.setDirection(DcMotorSimple.Direction.REVERSE);
        glisare.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_cutie.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare_perii.setDirection(DcMotorSimple.Direction.FORWARD);
        rotire_perii.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void firstGamepad()
    {
        if (abs(gamepad1.left_stick_y) > 0.1) mers_left.setPower(Range.clip(gamepad1.left_stick_y, -0.9, 0.9));
        else mers_left.setPower(0);

        if (abs(gamepad1.right_stick_y) > 0.1) mers_right.setPower(Range.clip(gamepad1.right_stick_y, -0.9, 0.9));
        else mers_right.setPower(0);
    }

    private void secondGamepad()
    {
        if ( gamepad2.dpad_up ) ridicare_cutie.setPower(0.5);
        else if ( gamepad2.dpad_down ) ridicare_cutie.setPower(-0.5);
        else ridicare_cutie.setPower(0);

        if ( gamepad2.a ) ridicare_perii.setPower(-0.4);
        else if ( gamepad2.b ) ridicare_perii.setPower(0.4);
        else ridicare_perii.setPower(0);

        if ( gamepad2.left_bumper ) rotire_perii.setPower(-0.4);
        else if ( gamepad2.right_bumper ) rotire_perii.setPower(0.4);
        else rotire_perii.setPower(0);

        if (abs(gamepad2.left_stick_y) > 0.1) glisare.setPower(Range.clip(gamepad2.left_stick_y, -0.9, 0.9));
        else glisare.setPower(0);
     }
}