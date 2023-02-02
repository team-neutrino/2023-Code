package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class PoseProcessor {
    
    private String m_poseFilepath;
    private ArrayList<PoseTriplet> m_poseArray;

    public PoseProcessor(String p_poseFilepath) {
        m_poseFilepath = p_poseFilepath;
        readFile();
    }

    private void readFile() {
        try {
            File poseFile = new File(m_poseFilepath);
            Scanner scanner = new Scanner(poseFile);
            while (scanner.hasNextLine()) {
                String fullLine = scanner.nextLine();
                String[] parsedLine = fullLine.split(",");
                Double[] coordList = new Double[3];
                for(int i = 0; i < 3; ++i)
                    coordList[i] = Double.valueOf(parsedLine[i]);
                PoseTriplet poseTriplet =
                    new PoseTriplet(coordList[0], coordList[1], coordList[2]);
                m_poseArray.add(poseTriplet);
            }
        } catch(FileNotFoundException e) {
            System.out.println("Error from PoseProcessor.java");
            e.printStackTrace();
        }
    }

    public ArrayList<PoseTriplet> getPoseArray() {
        return m_poseArray;
    }

}
