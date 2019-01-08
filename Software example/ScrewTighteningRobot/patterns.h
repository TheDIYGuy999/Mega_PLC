//
// =======================================================================================================
// DEFINE SCREW PATTERN ARRAYS
// =======================================================================================================
//

// PATTERN #1 (Program 1)For torque calibration-------
const unsigned int numberOfScrews1 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos1[numberOfScrews1][2] PROGMEM = {
  {0, 0}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
};

// PATTERN #2 (Program 2) -----
const unsigned int numberOfScrews2 = 5; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos2[numberOfScrews2][2] PROGMEM = {
  {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
  {0, 29.7}, // Screw #2
  {60, 0}, // Screw #3
  {60, 29.7}, // Screw #4
};

// PATTERN #3 (Program 3) ---------
const unsigned int numberOfScrews3 = 7; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos3[numberOfScrews3][2] PROGMEM = {
  {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 43.2}, // Screw #1   X, Y Position
  {0, 25.2}, // Screw #2
  {0, 0},
  {60, 0},
  {60, 25.2}, 
  {60, 43.2},
};

// PATTERN #4 (Program 4) ---------
const unsigned int numberOfScrews4 = 7; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos4[numberOfScrews4][2] PROGMEM = {
  {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 47.9}, // Screw #1   X, Y Position
  {0, 27.4}, // Screw #2
  {0, 0}, // etc...
  {60, 0},
  {60, 27.4},
  {60, 47.9},
};

// PATTERN #5 (Program 5) ---------
const unsigned int numberOfScrews5 = 5; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos5[numberOfScrews5][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
  {0, 7}, // Screw #2
  {60, 0}, // etc...
  {60, 7},
};

// PATTERN #6 (Program 6) ---------
const unsigned int numberOfScrews6 = 7; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos6[numberOfScrews6][2] PROGMEM = {
   {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 15.8}, // Screw #1   X, Y Position
  {0, 0}, // Screw #2
  {0, 45.4}, // etc...
  {60, 15.8},
  {60, 0},
  {60, 45.4},
};

// PATTERN #7 (Program 7) ---------
const unsigned int numberOfScrews7 = 7; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos7[numberOfScrews7][2] PROGMEM = {
 {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 20.5}, // Screw #1   X, Y Position
  {0, 0}, // Screw #2
  {0, 47.9}, // etc...
  {60, 20.5},
  {60, 0},
  {60, 47.9},
};

// PATTERN #8 (Program 8) ---------
const unsigned int numberOfScrews8 = 5; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos8[numberOfScrews8][2] PROGMEM = {
  {30, 100}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
  {0, 25.3}, // Screw #2
  {60, 0}, // etc...
  {60, 25.3},
};

// PATTERN #9 (Program 9) ----------
const unsigned int numberOfScrews9 = 3; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos9[numberOfScrews9][2] PROGMEM = {
   {18, 50}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
  {36.1, 0}, // Screw #2
};

// PATTERN #10 (Program 10)(Reserve)------------------------------
const unsigned int numberOfScrews10 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos10[numberOfScrews10][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #11 (Program 12)(Reserve)------------------------------
const unsigned int numberOfScrews11 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos11[numberOfScrews11][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #12 (Program 12)(Reserve)------------------------------
const unsigned int numberOfScrews12 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos12[numberOfScrews12][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #13 (Program 13)(Reserve)------------------------------
const unsigned int numberOfScrews13 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos13[numberOfScrews13][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #14 (Program 14)(Reserve)------------------------------
const unsigned int numberOfScrews14 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos14[numberOfScrews14][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #15 (Program 15)(Reserve)------------------------------
const unsigned int numberOfScrews15 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos15[numberOfScrews15][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #16 (Program 16)(Reserve)------------------------------
const unsigned int numberOfScrews16 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos16[numberOfScrews16][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #17 (Program 17)(Reserve)------------------------------
const unsigned int numberOfScrews17 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos17[numberOfScrews17][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #18 (Program 18)(Reserve)------------------------------
const unsigned int numberOfScrews18 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos18[numberOfScrews18][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #19 (Program 19)(Reserve)------------------------------
const unsigned int numberOfScrews19 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos19[numberOfScrews19][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };

 // PATTERN #20 (Program 20)(Reserve)------------------------------
const unsigned int numberOfScrews20 = 2; // Number of Screws + 1 (for Cycle End Position) = size of array!

const float pos20[numberOfScrews20][2] PROGMEM = {
  {30, 65}, // Screw #0 (Cycle End)  {X, Y} = X, Y Position

  {0, 0}, // Screw #1   X, Y Position
 };
