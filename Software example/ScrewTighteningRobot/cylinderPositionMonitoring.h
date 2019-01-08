// This class can be called for each cylinder in the system, in order to generate cylinder position error messages

// Header---------------------------------------------------------
class cylinderPosition {
  public:
    cylinderPosition();
    boolean monitoringRlAl(int output, int sensorRL, int sensorAL);
  private:
    int _output;
    int _sensorRL;
    int _sensorAL;
    unsigned long _now;
};

// Code------------------------------------------------------------
cylinderPosition::cylinderPosition() {
}
boolean cylinderPosition::monitoringRlAl(int output, int sensorRL, int sensorAL) {
  _output = digitalRead(output);
  _sensorRL = digitalRead(sensorRL);
  _sensorAL = digitalRead(sensorAL);

  if (_output && _sensorAL && !_sensorRL || !_output && _sensorRL && !_sensorAL) { // if end position is OK
    _now = millis(); // reset error delay time
  }

  if (millis() > _now + 2000) { // if position > 2s not OK
    return true;
  } else {
    return false;
  }
}


