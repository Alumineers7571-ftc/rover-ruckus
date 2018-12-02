package org.firstinspires.ftc.teamcode.Hardware.Recording;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@Autonomous(name = "Replayer", group = "test")
@Disabled
public class InputReplayer extends OpMode{

    DriveTrain dt = new DriveTrain();

    String fileName = null;
    String line = null;

    enum State {
        TRANSLATELINE, FEEDTOROBO
    }

    State state;

    //double[][] powerValues;

    double[] leftPowerValues;
    double[] rightPowerValues;

    double ly, rx;
    //double lx, ry;

    int countLines, countReplays;

    File file;

    FileReader fileReader;
    BufferedReader bufferedReader;

    String[] tempString = new String[20];

    @Override
    public void init() {

        telemetry.addData("String: ", tempString);
        telemetry.addData("ly: ", ly);
        telemetry.addData("rx: ", rx);
        //telemetry.addData("powerValues: ", leftPowerValues.toString() + "    " + rightPowerValues.toString());

        //fileName = FinalValues.fileName;

        file = new File("/storage/emulated/0/InputFiles/inputTestNew.txt");

        countLines = 0;
        countReplays = 0;

        //dt.init(hardwareMap, telemetry);

        leftPowerValues = new double[1500];
        rightPowerValues = new double[1500];

        try {

            fileReader = new FileReader(file);
            bufferedReader = new BufferedReader(fileReader);



            while ((line = bufferedReader.readLine()) != null) {

                tempString = line.split(";");

                ly = Double.parseDouble(tempString[0]);
                rx = Double.parseDouble(tempString[1]);

                leftPowerValues[countLines] = Range.clip(ly + rx, -1, 1);
                rightPowerValues[countLines] = Range.clip(ly - rx, -1, 1);

                countLines++;
                telemetry.update();
            }

            bufferedReader.close();

            countReplays = 0;
        }
        catch(IOException ex) {
            ex.printStackTrace();
        }

    }

    @Override
    public void loop() {

        telemetry.addData("powerValues: ", leftPowerValues[countReplays] + "    " + rightPowerValues[countReplays]);

        dt.runMotorsSides(leftPowerValues[countReplays], rightPowerValues[countReplays]);

        telemetry.update();

        if (countReplays >= countLines){
            stop();
        } else {
            countReplays++;
        }
    }

    @Override
    public void stop(){ }
}
