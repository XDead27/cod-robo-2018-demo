package org.firstinspires.ftc.teamcode;

public final class AutonomousSquareOneSensor extends AutonomousTest {

    @Override
    protected void runOp() {
        switch(ChooseAndPushCube(false)){
            //TODO: scoate sleep urile
            case 1:
                //cub este in fata
                //se duce in craterul inamic
                //TODO : sa nu dea in mingie

                sleep(1000);

                mers_encoder(25, 0.7);

                ThrowTotem();

                rotit(90);

                walk_with_obstacle_and_range(15, false);

                rotit(90);
                break;

            case 2:
                //cub este in stanga
                //se duce in crater inamic

                sleep(1000);

                rotit(30);

                walk_with_obstacle_and_range(20, false);

                rotit(-90);

                ThrowTotem();

                rotit(180);
                break;

            case 3:
                //cub este in dreapta
                //se duce in crater aliat

                sleep(1000);

                walk_with_obstacle_and_range(15, false);

                rotit(90);

                ThrowTotem();

                rotit(180);

                break;
        }

        sleep(1000);

        walk_with_obstacle_and_range(20, false);

        stopWheels();
    }

    @Override
    protected void endOp() {

    }
}
