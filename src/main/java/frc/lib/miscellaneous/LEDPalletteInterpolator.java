// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.lib.miscellaneous;

// import edu.wpi.first.math.MathUtil;

// /** Add your docs here. */
// public class LEDPalletteInterpolator {
//   // the length of the led strip
//   int stripLength;
//   // the minimum value of the base unit. This is used in interpolation. For the
//   // fire LEDs, this is temperature.
//   int minVal;
//   // the maximum value of the base unit. This is used in interpolation. For the
//   // fire LEDs, this is temperature.
//   int maxVal;
//   // number of colors in the pallete
//   int numOfColors;
//   // an array of the colors in the pallete
//   int[][] colors;

//   /**
//    * Color converter for pallete based LEDs.
//    * 
//    * @param stripLength the length of the led strip
//    * @param minVal      the minimum value of the base unit. This is used in
//    *                    interpolation. For the fire LEDs, this is temperature.
//    * @param maxVal      the maximum value of the base unit. This is used in
//    *                    interpolation. For the fire LEDs, this is temperature.
//    * @param colors      The colors of the desired pallete in the form of an int
//    *                    array. ONLY PASS INTEGER ARRAYS OF LENGTH 4. ARRAYS PASSED
//    *                    MUST BE ORDERED BY BASE UNIT VALUE FROM LOWEST TO HIGHEST.
//    *                    The first 3 values are the RGB values of the color. The
//    *                    final value is the base unit value associated with that
//    *                    color (temperature value for fire)
//    */
//   public LEDPalletteInterpolator(int stripLength, int minVal, int maxVal, int[]... colors) {
//     this.stripLength = stripLength;
//     this.colors = colors;
//     numOfColors = colors.length;
//   }

//   /**
//    * Converts the given base unit value into an interpolated RGB value based on
//    * the pallete
//    * 
//    * @param value base unit.
//    * 
//    * @return the calculated color value in terms of an RBG int array
//    */
//   public int[] convertToPallete(double value) {
//     // find the bounds of the interpolation
//     int startIndex = 0;
//     int endIndex = 0;
//     double interpolationRatio = 0;

//     for (int i = 0; i < colors.length - 1; i++) {
//       if (value > colors[i][3]) {
//         startIndex = i;
//         endIndex = i + 1;
//       }
//     }

//     interpolationRatio = (double)(value - colors[startIndex][3]) / (double)(colors[endIndex][3] - colors[startIndex][3]); 

//     return new int[(int) MathUtil.interpolate(colors[startIndex][0], colors[endIndex][0], interpolationRatio),
//                    (int) MathUtil.interpolate(colors[startIndex][1], colors[endIndex][1], interpolationRatio),
//                    (int) MathUtil.interpolate(colors[startIndex][2], colors[endIndex][2], interpolationRatio)];

//   }

// }
