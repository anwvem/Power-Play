package org.firstinspires.ftc.teamcode.glowCode;

import com.vuforia.Image;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.io.File;

import java.io.FileNotFoundException;
import java.io.IOException;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvWebcam;

public class DetectingSignalSleeve extends HardwareMapping {

    public int[][][] calculateData()
    {
        //I am going to shamelessly steal from vuforia

        final int ROWS = 1280;
        final int COLUMNS = 720;
        int[][][] picture = new int[ROWS][COLUMNS][3];

        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLUMNS; j++) {
                //This should detect the Red in Red

                //This should detect the Green

                //This should detect the Blue

            }
        }
        return picture;
    }
}
