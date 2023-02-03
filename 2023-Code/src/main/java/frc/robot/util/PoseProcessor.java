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
    // getPoseArray();
  }

  private void readFile() {
    try {
      File poseFile = new File("C:\\Users\\neutr\\Documents\\GitHub\\2023-Code\\2023-Code\\src\\main\\java\\frc\\robot\\util\\testFile.txt");
      Scanner scanner = new Scanner(poseFile);
      while (scanner.hasNextLine()) {
        String fullLine = scanner.nextLine();
        System.out.println(fullLine);
        String[] parsedLine = fullLine.split(",");
        Double[] coordList = new Double[3];
        for (int i = 0; i < 3; ++i) coordList[i] = Double.valueOf(parsedLine[i]);
        PoseTriplet poseTriplet = new PoseTriplet(coordList[0], coordList[1], coordList[2]);
        m_poseArray.add(poseTriplet);
      }
    } catch (FileNotFoundException e) {
      System.out.println("Error from PoseProcessor.java");
      e.printStackTrace();
    }
  }

  public ArrayList<PoseTriplet> getPoseArray() {
    System.out.println("test");
    for (PoseTriplet p : m_poseArray) {
      System.out.println(
          "Coord1: " + p.getCoord1() + ", Coord2: " + p.getCoord2() + ", Coord3: " + p.getAngle());
    }
    return m_poseArray;
  }
}
