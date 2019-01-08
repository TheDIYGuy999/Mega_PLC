// =======================================================================================================
// DEFINE YXZ OFFSETS FOR ALL 10 SCREW PATTERNS
// =======================================================================================================
//

// Offset values are specified as X, Y, Z coordinates from the reference switch to the first screw in the pattern

float offset[numberOfPrograms][3] = {
  {144, 110, -75.5}, // Program #1 (for torque calibration)
  {61.7, 65, -104},  // Program #2
  {62, 65, -104},  // Program #3
  {62, 65, -104},  // Program #4
  {62, 101, -104}, // Program #5
  {62, 65, -104},  // Program #6
  {62, 64.7, -104}, // Program #7
  {62, 65, -104},  // Program #8
  {74, 110, -104},  // Program #9
  {62, 65, -104},  // Program #10 Reserve
  {62, 65, -104},  // Program #11 Reserve
  {62, 65, -104},  // Program #12 Reserve
  {62, 65, -104},  // Program #13 Reserve
  {62, 65, -104},  // Program #14 Reserve
  {62, 65, -104},  // Program #15 Reserve
  {62, 65, -104},  // Program #16 Reserve
  {62, 65, -104},  // Program #17 Reserve
  {62, 65, -104},  // Program #18 Reserve
  {62, 65, -104},  // Program #19 Reserve
  {62, 65, -104}   // Program #20 Reserve
};
