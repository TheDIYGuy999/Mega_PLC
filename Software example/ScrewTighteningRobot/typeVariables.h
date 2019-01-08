//
// =======================================================================================================
// TYPE SPECIFIC Z AXIS VARIABLES, SCREW PITCH, SCREW RPM
// =======================================================================================================
//

void typeVariables() {
  switch (activeProgram) {
    case 1: // Program #1 (für Drehmoment Kalibrierung)
      // Define Z Axis heights
      zError = 75;
      zRetract = 20;
      zSuck = 20;
      zScrewBegin = -5.0;
      zScrewingDepth = 12.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews1;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 0;
      spindleDegreesMax = 6000; // 6000° = limit inactive!
      break;

    case 2: // Program #2 
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews2;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 3: // Program #3 
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -7.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews3;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 4: // Program #4 
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews4;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 5: // Program #5
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews5;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 6: // Program #6
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews6;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 7: // Program #7
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews7;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 8: // Program #8
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews8;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

    case 9: // Program #9
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews9;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 750;
      spindleDegreesMax = 6000;
      break;

    case 10: // Program #10
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews10;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 11: // Program #11
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews11;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 12: // Program #12
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews12;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 13: // Program #13
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -7.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews13;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 14: // Program #14
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews14;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 15: // Program #15
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews15;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 16: // Program #16
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -7.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews16;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 17: // Program #17
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews17;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 18: // Program #18
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews18;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 19: // Program #19
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews19;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;

      case 20: // Program #20
      // Define Z Axis heights
      zError = 100;
      zRetract = 50;
      zSuck = 20;
      zScrewBegin = -8.0;
      zScrewingDepth = 9.0;
      // Define number of screws in the pattern
      numberOfScrews = numberOfScrews20;
      // Define screw pitch
      screwPitch = 1.12;
      // Define screw RPM
      screwRpm = 400;
      // screwing rotation angle tolerances
      spindleDegreesMin = 800;
      spindleDegreesMax = 6000;
      break;
  }
}
