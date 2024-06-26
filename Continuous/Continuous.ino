/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/
// Define the states using an enum
typedef enum {
    LANDED,
    FLYING
} DroneState;


#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

DroneState previousState = LANDED;
DroneState currentState = LANDED;

long sensorValue = 0;
long distanceToGroundFromBottomOfDrone = 0;

void updateState(DroneState newState){
  previousState = currentState;
  currentState = newState;
}

void updateSensorValue(){
  sensorValue = sensor.read();

  distanceToGroundFromBottomOfDrone = sensorValue - 200;

  if(distanceToGroundFromBottomOfDrone < 0){
    distanceToGroundFromBottomOfDrone = 0;
  }
}



// Function prototypes
void handleLandedState();
void handleFlyingState();

// State machine function
void droneStateMachine(DroneState *state);

void handleFlyingState() {
  if (distanceToGroundFromBottomOfDrone < 10){
    updateState(LANDED);
  } else {
    if (distanceToGroundFromBottomOfDrone < 1000){
      int delayTime = (distanceToGroundFromBottomOfDrone / 3);
  
      tone(7,400);
      delay(50);
      noTone(7);
      delay(delayTime);
  
      updateState(FLYING);
    }
  }
}

void handleLandedState() {
  if(previousState == FLYING){
    tone(7,400);
    delay(1000);
    noTone(7);
    updateState(LANDED);
  } else {
    if (distanceToGroundFromBottomOfDrone > 50){
      updateState(FLYING);
    } else {
      updateState(LANDED);
    }
  }
}


void droneStateMachine() {
    switch (currentState) {
        case LANDED:
            handleLandedState();
            break;
        case FLYING:
            handleFlyingState();
            break;
        default:
            printf("Invalid state!\n");
            break;
    }
}

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
  pinMode(7, OUTPUT);

  digitalWrite(7, LOW);
}

void loop()
{
  updateSensorValue();
  Serial.println("start");
  Serial.println(sensorValue);
  Serial.println(distanceToGroundFromBottomOfDrone);

  if (currentState == LANDED){
    Serial.println("LANDED");
  } else {
    Serial.println("FLYING");
  }

   droneStateMachine();
  
  delay(100);
}
