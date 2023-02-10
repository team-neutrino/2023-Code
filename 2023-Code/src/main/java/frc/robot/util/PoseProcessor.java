package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

import frc.robot.Constants;

public class PoseProcessor {

  public static String absoluteAppend(String p_filename) {
    String ret = System.getProperty("user.dir") + Constants.UtilConstants.UNIVERSAL_DIRECTORY + p_filename;
    //System.out.println(ret); 
    return ret;
  }

  public static ArrayList<PoseTriplet> poseTripletsFromFile(String p_filename) {
    ArrayList<PoseTriplet> poseArray = new ArrayList<PoseTriplet>();
    try {
      File poseFile = new File(absoluteAppend(p_filename));
      File outputFile = new File(absoluteAppend("testOutput.txt"));
      Scanner scanner = new Scanner(poseFile);
      try {
        BufferedWriter bf = new BufferedWriter(new FileWriter(outputFile, true));
        bf.write("test\n");
        bf.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
      while (scanner.hasNextLine()) {
        String fullLine = scanner.nextLine();
        String[] parsedLine = fullLine.split(",");
        Double[] coordList = new Double[3];
        for (int i = 0; i < 3; ++i) coordList[i] = Double.valueOf(parsedLine[i]);
        PoseTriplet poseTriplet = new PoseTriplet(coordList[0], coordList[1], coordList[2]);
        poseArray.add(poseTriplet);
      }
    } catch (FileNotFoundException e) {
      System.out.println("Error from PoseProcessor.java");
      e.printStackTrace();
    }
    return poseArray;
  }
}
