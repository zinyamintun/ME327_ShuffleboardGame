/****************************************************************************
  Module
   checkPos.ino

  Revision
   1.0.0

  Description

  Notes

  History
  When           Who     What/Why
  -------------- ---     --------
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Libraries
#include <math.h>
#include <SoftwareSerial.h>
#include <ezButton.h>

/*----------------------------- Module Defines ----------------------------*/
#define BAUD_RATE 115200
#define MAX_DUTY_CYCLE 255
// Aliases for convenience
#define D1 (mag_24)
#define D2 (mag_2H)
#define D3 (mag_H3)
#define xh (P3x)
#define yh (P3y)
/*---------------------------- Module Functions ---------------------------*/
void getPositions(void);
void getKinematics(void);
void setDirections(void);
void setDutyCycles(void);
void setPwmFrequency(int pin, int divisor);
void readButton(void);
void sendValues(void);
/*---------------------------- Module Variables ---------------------------*/
// Pin declares
int left_sensorPosPin = A2; // input pin for left motor MR sensor
int right_sensorPosPin = A5;

int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int pwmPin2 = 6; // PWM output pin for motor 2 
int dirPin2 = 7;

int buttonPin = 2;
ezButton button(buttonPin); // Use ezButton for debouncing, pin 2

// Position Tracking: Left Motor
static int left_updatedPos = 0;      // keeps track of the latest updated value of the MR sensor reading
static int left_rawPos = 0;          // current raw reading from MR sensor
static int left_lastRawPos = 0;      // last raw reading from MR sensor
static int left_lastLastRawPos = 0;  // last last raw reading from MR sensor
static int left_flipNumber = 0;      // keeps track of the number of flips over the 180deg mark
static int left_tempOffset = 0;
static int left_rawDiff = 0;
static int left_lastRawDiff = 0;
static int left_rawOffset = 0;
static int left_lastRawOffset = 0;

const int left_flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
static bool left_flipped = false;

const double left_m_trend = -0.0141; //-0.0201;  // slope
const double left_b_trend = 4.75; // -31.7962;    // y-intercept

// Position Tracking: Right Motor
static int right_updatedPos = 0;      // keeps track of the latest updated value of the MR sensor reading
static int right_rawPos = 0;          // current raw reading from MR sensor
static int right_lastRawPos = 0;      // last raw reading from MR sensor
static int right_lastLastRawPos = 0;  // last last raw reading from MR sensor
static int right_flipNumber = 0;      // keeps track of the number of flips over the 180deg mark
static int right_tempOffset = 0;
static int right_rawDiff = 0;
static int right_lastRawDiff = 0;
static int right_rawOffset = 0;
static int right_lastRawOffset = 0;

const int right_flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
static bool right_flipped = false;

const double right_m_trend = 0.0141; //0.0222;  // slope
const double right_b_trend = -5.35; //41.9977;    // y-intercept

// Kinematics ---------------------------------
// Pantograph dimensions
const double a1 = 0.157;   //[m]
const double a2 = 0.1724;  //[m]
const double a3 = 0.1724;  //[m]
const double a4 = 0.157;   //[m]
const double a5 = 0.0;     //[m]
const double thetaOffset = 49.2; //[deg]
static double thetaLeft; //[deg]
static double thetaRight; //[deg]
static double theta1;      //[deg]   //measured from sensors
static double theta5;      //[deg]   //measured from sensors

//---------------------------------------------
const double rPuck = 0.02;         //[m]
static double xProxy;
static double yProxy;

// Boundary wall constants 

static const double EWall = 0.12;  //[m]
static const double SWall = 0.04;              //[m]
static const double WWall = -0.13;       //[m]
static const double NWall = 0.32; //[m]

static const double kWall = 1500;  //[N/m]
static const double skWall = 1200;

// Pantograph Kinematics Calculation-----------------------------
static double P2x; // Set Positions of Points 2 and 4
static double P2y;
static double P4x;
static double P4y;

static double mag_24; //Distance from 2 to 4
static double mag_2H; //Distance from 2 to PH
static double mag_H3; //Distance from PH to 3 (end effector position)

static double PHx; // Set position of PH
static double PHy;

static double P3x; // Set Position of 3 (End effector)
static double P3y;

// Calculate Partials for Jacobian
static double d1x2;
static double d1y2;
static double d1y4;
static double d1x4;
static double d1D1;
static double d1D2;
static double d1D3;
static double d1yh;
static double d1xh;
static double d1y3;
static double d1x3;

static double d5x4;
static double d5y4;
static double d5y2;
static double d5x2;
static double d5D1;
static double d5D2;
static double d5D3;
static double d5yh;
static double d5xh;
static double d5y3;
static double d5x3;

// For dynamics and force implementation
// const double mPuck = 1;  //[kg]
const double dt = 0.01; //[s]

static double xh_prev;          // x displacement
static double dxh;              // x velocity of the puck
static double dxh_prev;
static double dxh_prev2;
static double dxh_filt;         // x filtered velocity of the handle
static double dxh_filt_prev;
static double dxh_filt_prev2;
static double axh = 0;
static double axh_prev = 0;
static double axh_prev2 = 0;
static double axh_filt = 0;         // x filtered acceleration of the puck
static double axh_filt_prev = 0;
static double axh_filt_prev2 = 0;

static double yh_prev;          // y displacement
static double dyh;              // y velocity of the puck
static double dyh_prev;
static double dyh_filt;         // y filtered velocity of the handle
static double dyh_filt_prev;
static double ayh = 0;
static double ayh_prev = 0;
static double ayh_filt = 0;         // y filtered acceleration of the puck
static double ayh_filt_prev = 0;

static const double rs = 0.075;    //0.070;   //[m]
static const double rp = 0.00475;  //0.0095;  //[m]
static double currXForce = 0;
static double prevXForce = 0;
static double currYForce = 0;
static double prevYForce = 0;
static double Fx;
static double Fy;
static double FxPuck;
static double FyPuck;
static double T1;
static double T5;
double duty1 = 0;            // duty cylce (between 0 and 255)
unsigned int output1 = 0;    // output command to the motor
double duty2 = 0;            // duty cylce (between 0 and 255)
unsigned int output2 = 0;    // output command to the motor

// Motors ---------------------------------
static double dutyCycle_Motor1;
static double dutyCycle_Motor2;

static const double DC_divisor = 0.0183;

// Button-----------------------------------
int buttonState;
int prevButtonState;
int loopCounter;

// Frictions--------------------------------
static double friction_x = 0;
static double friction_y = 0;
static double frictionRange = 0.5;
static double frictionBoundary = 0.75;
static double slope = frictionBoundary/ frictionRange;

// Inertia--------------------------------
static double v_xProxy = 0;
static double v_yProxy = 0;
static double a_xProxy = 0;
static double a_yProxy = 0;
const double mPuck = 0.55;  //[kg]
const double km = 80;
const double bm = 2*sqrt(km *mPuck);

// === Proxy Inertia Model Parameters ===
const double maxOffset = 0.10;  // limit how far proxy can drift from user [m]
// === Store Previous Values for Trapezoidal Integration ===
static double last_v_xProxy = 0;
static double last_v_yProxy = 0;
static double last_a_xProxy = 0;
static double last_a_yProxy = 0;

/*------------------------------ Module Code ------------------------------*/
void setup() 
{
  // Set up serial communication
  Serial.begin(BAUD_RATE);

  // Initialize position valiables
  left_lastLastRawPos = analogRead(left_sensorPosPin);
  left_lastRawPos = analogRead(left_sensorPosPin);
  right_lastLastRawPos = analogRead(right_sensorPosPin);
  right_lastRawPos = analogRead(right_sensorPosPin);

  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  setPwmFrequency(pwmPin2,1);
    // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  pinMode(pwmPin2, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin2, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  analogWrite(pwmPin2, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin2, LOW);  // set direction

  // Initialize button
  pinMode(buttonPin, INPUT_PULLUP); 
  button.setDebounceTime(50);

  xProxy = xh;
  yProxy = yh;
  
}

void loop(){
  getPositions();
  getKinematics();
  // getDynamics();
  setDirections();
  setDutyCycles();
  readButton();

  loopCounter++;

  // Send data only every 10 loops
  if (loopCounter >= 20) {
    sendValues();
    loopCounter = 0;  
  }

  if (buttonState == 1){
    getDynamics();
  }
  else{
    resetForce();
  }



  // Serial.print("xh: ");
  // Serial.print(xh,5);
  // Serial.print("| yh:");
  // Serial.println(yh,5);
  // Serial.print(xh, 5); 
  // Serial.print(",");
  // Serial.println(yh, 5);
  // Serial.println(yh, 5);
   //for calibration:
  // Serial.print("thetaLeft: ");
  // Serial.print(thetaLeft,4);
  // Serial.print("| thetaRight:");
  // Serial.println(thetaRight,4);

  // Serial.print("axh_filt: ");
  // Serial.print(axh_filt);
  // Serial.print("| ayh_filt:");
  // Serial.println(ayh_filt);

}


/***************************************************************************
   @Function: getPositions
   @Arguments: none
   @Returns: none
   @Description:
 ***************************************************************************/
void getPositions(void) {
  //*************************************************************
  //*** Section 1. Compute position in counts *******************
  //*************************************************************
  // Get voltage output by MR sensor
  left_rawPos = analogRead(left_sensorPosPin);  //current raw position from MR sensor
  right_rawPos = analogRead(right_sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  left_rawDiff = left_rawPos - left_lastRawPos;          //difference btwn current raw position and last raw position
  left_lastRawDiff = left_rawPos - left_lastLastRawPos;  //difference btwn current raw position and last last raw position
  left_rawOffset = abs(left_rawDiff);
  left_lastRawOffset = abs(left_lastRawDiff);

  right_rawDiff = right_rawPos - right_lastRawPos;          //difference btwn current raw position and last raw position
  right_lastRawDiff = right_rawPos - right_lastLastRawPos;  //difference btwn current raw position and last last raw position
  right_rawOffset = abs(right_rawDiff);
  right_lastRawOffset = abs(right_lastRawDiff);

  // Update position record-keeping vairables
  left_lastLastRawPos = left_lastRawPos;
  left_lastRawPos = left_rawPos;

  right_lastLastRawPos = right_lastRawPos;
  right_lastRawPos = right_rawPos;

  // Keep track of flips over 180 degrees
  if ((left_lastRawOffset > left_flipThresh) && (!left_flipped)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (left_lastRawDiff > 0) {                                     // check to see which direction the drive wheel was turning
      left_flipNumber--;                                            // cw rotation
    } else {                                                        // if(rawDiff < 0)
      left_flipNumber++;                                            // ccw rotation
    }
    if (left_rawOffset > left_flipThresh) {                              // check to see if the data was good and the most current offset is above the threshold
      left_updatedPos = left_rawPos + left_flipNumber * left_rawOffset;  // update the pos value to account for flips over 180deg using the most current offset
      left_tempOffset = left_rawOffset;
    } else {                                                                 // in this case there was a blip in the data and we want to use lastactualOffset instead
      left_updatedPos = left_rawPos + left_flipNumber * left_lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      left_tempOffset = left_lastRawOffset;
    }
    left_flipped = true;                                                // set boolean so that the next time through the loop won't trigger a flip
  } else {                                                              // anytime no flip has occurred
    left_updatedPos = left_rawPos + left_flipNumber * left_tempOffset;  // need to update pos based on what most recent offset is
    left_flipped = false;
  }

  if ((right_lastRawOffset > right_flipThresh) && (!right_flipped)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (right_lastRawDiff > 0) {                                       // check to see which direction the drive wheel was turning
      right_flipNumber--;                                              // cw rotation
    } else {                                                           // if(rawDiff < 0)
      right_flipNumber++;                                              // ccw rotation
    }
    if (right_rawOffset > right_flipThresh) {                                // check to see if the data was good and the most current offset is above the threshold
      right_updatedPos = right_rawPos + right_flipNumber * right_rawOffset;  // update the pos value to account for flips over 180deg using the most current offset
      right_tempOffset = right_rawOffset;
    } else {                                                                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      right_updatedPos = right_rawPos + right_flipNumber * right_lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      right_tempOffset = right_lastRawOffset;
    }
    right_flipped = true;                                                   // set boolean so that the next time through the loop won't trigger a flip
  } else {                                                                  // anytime no flip has occurred
    right_updatedPos = right_rawPos + right_flipNumber * right_tempOffset;  // need to update pos based on what most recent offset is
    right_flipped = false;
  }

  //*************************************************************
  //*** Section 2. Compute angles in degrees ********************
  //*************************************************************
  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  thetaLeft = (left_m_trend * (double)left_updatedPos) + left_b_trend;  // linear trendline of sector angle vs. updatedPos
  thetaRight = (right_m_trend * (double)right_updatedPos) + right_b_trend;  // linear trendline of sector angle vs. updatedPos

  // Serial.print(" | left_updatedPos:");
  // Serial.println(left_updatedPos);
  // Serial.print(" | right_updatedPos:");
  // Serial.println(right_updatedPos);

  // Serial.print("thetaLeft:");
  // Serial.print(thetaLeft);
  // Serial.print(" | thetaRight:");
  // Serial.println(thetaRight);

  // Calculate theta1 and theta5
  theta1 = thetaLeft + thetaOffset;
  theta5 = thetaRight - thetaOffset + 180;
  // Serial.print("theta1:");
  // Serial.print(theta1);
  // Serial.print(" | theta5:");
  // Serial.println(theta5);
}

/***************************************************************************
   @Function: getKinematics
   @Arguments: none
   @Returns: none
   @Description:
 ***************************************************************************/
void getKinematics(void) {
  // Set Positions of Points 2 and 4
  theta1 = theta1 / 180 * PI;
  theta5 = theta5 / 180 * PI;
  P2x = a1 * cos(theta1);
  P2y = a1 * sin(theta1);
  P4x = a4 * cos(theta5) - a5;
  P4y = a4 * sin(theta5);

  //Distance from 2 to 4
  mag_24 = sqrt((P2x - P4x) * (P2x - P4x) + (P2y - P4y) * (P2y - P4y));
  //Distance from 2 to PH
  mag_2H = (a2 * a2 - a3 * a3 + mag_24 * mag_24) / (2 * mag_24);  // calculation doesnt seem to match reality
  //Distance from PH to 3 (end effector position)
  mag_H3 = sqrt(a2 * a2 - mag_2H * mag_2H);

  // Set position of PH
  PHx = P2x + mag_2H / mag_24 * (P4x - P2x);
  PHy = P2y + mag_2H / mag_24 * (P4y - P2y);

  // Set Position of 3 (End effector)
  P3x = PHx + mag_H3 / mag_24 * (P4y - P2y);
  P3y = PHy + mag_H3 / mag_24 * (P2x - P4x);

  // Calculate Partials for Jacobian
  d1x2 = -a1 * sin(theta1);
  d1y2 = a1 * cos(theta1);
  d1y4 = 0;
  d1x4 = 0;
  d1D1 = ((P4x - P2x) * (d1x4 - d1x2) + (P4y - P2y) * (d1y4 - d1y2)) / D1;
  d1D2 = d1D1 - d1D1 * (a2 * a2 - a3 * a3 + D1 * D1) / (2 * D1 * D1);
  d1D3 = -D2 * d1D2 / D3;
  d1yh = d1y2 + (d1D2 * D1 - d1D1 * D2) * (P4y - P2y) / (D1 * D1) + D2 / D1 * (d1y4 - d1y2);
  d1xh = d1x2 + (d1D2 * D1 - d1D1 * D2) * (P4x - P2x) / (D1 * D1) + D2 / D1 * (d1x4 - d1x2);
  d1y3 = d1yh - D3 / D1 * (d1x4 - d1x2) - (d1D3 * D1 - d1D1 * D3) / (D1 * D1) * (P4x - P2x);
  d1x3 = d1xh + D3 / D1 * (d1y4 - d1y2) + (d1D3 * D1 - d1D1 * D3) / (D1 * D1) * (P4y - P2y);

  d5x4 = -a4 * sin(theta5);
  d5y4 = a4 * cos(theta5);
  d5y2 = 0;
  d5x2 = 0;
  d5D1 = ((P4x - P2x) * (d5x4 - d5x2) + (P4y - P2y) * (d5y4 - d5y2)) / D1;
  d5D2 = d5D1 - d5D1 * (a2 * a2 - a3 * a3 + D1 * D1) / (2 * D1 * D1);
  d5D3 = -D2 * d5D2 / D3;
  d5yh = d5y2 + (d5D2 * D1 - d5D1 * D2) * (P4y - P2y) / (D1 * D1) + D2 / D1 * (d5y4 - d5y2);
  d5xh = d5x2 + (d5D2 * D1 - d5D1 * D2) * (P4x - P2x) / (D1 * D1) + D2 / D1 * (d5x4 - d5x2);
  d5y3 = d5yh - D3 / D1 * (d5x4 - d5x2) - (d5D3 * D1 - d5D1 * D3) / (D1 * D1) * (P4x - P2x);
  d5x3 = d5xh + D3 / D1 * (d5y4 - d5y2) + (d5D3 * D1 - d5D1 * D3) / (D1 * D1) * (P4y - P2y);
}

/****************************************************************************
   @Function: getDynamics
   @Arguments: none
   @Returns: none
   @Description:
 ***************************************************************************/
void getDynamics(void) {
  // Calculate x velocity with loop time estimation
  dxh = (xh - xh_prev) / dt;
  // Calculate x acceleration with loop time estimation
  axh = (dxh - dxh_prev) / dt;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
  // Calculate filtered acceleration
  axh_filt = 0.9 * axh + 0.1 * axh_prev;

  // Record the position and velocity
  xh_prev = xh;
  dxh_prev = dxh;
  dxh_filt_prev = dxh_filt;
  axh_prev = axh;
  axh_filt_prev = axh_filt;

  // Calculate y velocity with loop time estimation
  dyh = (yh - yh_prev) / dt;
  // Calculate y acceleration with loop time estimation
  ayh = (dyh - dyh_prev) / dt;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dyh_filt = .9*dyh + 0.1*dyh_prev; 
  // Calculate filtered acceleration
  ayh_filt = 0.9 * ayh + 0.1 * ayh_prev;
  // Record the position and velocity
  yh_prev = yh;
  dyh_prev = dyh;
  dyh_filt_prev = dyh_filt;
  ayh_prev = ayh;
  ayh_filt_prev = ayh_filt;

  //////////////////////////////////////////Mass Inertia//////////////////////////////////////////////////
    // === Compute Spring Force from User Handle ===
  double FSpring_x = km * (xh - xProxy);
  double FSpring_y = km * (yh - yProxy);
  // === Damping Force (based on proxy velocity) ===
  double FDamping_x = bm * v_xProxy;
  double FDamping_y = bm * v_yProxy;
  // === Net Force on Virtual Object ===
  double ObjForce_x = FSpring_x - FDamping_x;
  double ObjForce_y = FSpring_y - FDamping_y;
  // === Compute Acceleration ===
  a_xProxy = ObjForce_x / mPuck;
  a_yProxy = ObjForce_y / mPuck;
  // === Velocity Update: Trapezoidal Integration ===
  v_xProxy += 0.5 * (last_a_xProxy + a_xProxy) * dt;
  v_yProxy += 0.5 * (last_a_yProxy + a_yProxy) * dt;
  // === Position Update: Trapezoidal Integration ===
  xProxy += 0.5 * (last_v_xProxy + v_xProxy) * dt;
  yProxy += 0.5 * (last_v_yProxy + v_yProxy) * dt;
  // === Optional: Limit Proxy Drift From Hand ===
  double max_dx = maxOffset;
  double max_dy = maxOffset;
  if (xProxy > xh + max_dx) xProxy = xh + max_dx;
  if (xProxy < xh - max_dx) xProxy = xh - max_dx;
  if (yProxy > yh + max_dy) yProxy = yh + max_dy;
  if (yProxy < yh - max_dy) yProxy = yh - max_dy;

  last_v_xProxy = v_xProxy;
  last_v_yProxy = v_yProxy;
  last_a_xProxy = a_xProxy;
  last_a_yProxy = a_yProxy;
  // === Output Force Feedback to Motors ===
  Fx = FSpring_x;
  Fy = FSpring_x;

  //////////////////////////////////////////Mass Inertia//////////////////////////////////////////////////

  //////////////////////////////////////////Wall//////////////////////////////////////////////////////////
  if (yh + rPuck > NWall){   //N 
    Fx += 0;
		Fy += -kWall*(NWall-(yh+rPuck));
		xProxy = xh;
    yProxy = NWall-rPuck;
    //Serial.println("N");
	}
	
	if(xh + rPuck > EWall){
		Fx += -skWall*(EWall-(xh+rPuck));
		Fy += 0;
		xProxy = EWall-rPuck;
    yProxy = yh;
    //Serial.println("E");
	}

	if(xh - rPuck < WWall){
		Fx += -skWall*(WWall-(xh-rPuck));
		Fy += 0;
		xProxy = WWall+rPuck;
    yProxy = yh;
  }
  //////////////////////////////////////////Wall//////////////////////////////////////////////////////////

  //////////////////////////////////////////Friction//////////////////////////////////////////////////////
  if (dxh_filt < frictionRange && dxh_filt > -frictionRange) {
    friction_x = - slope * dxh_filt;
  } else if (dxh_filt >= frictionRange) {
    friction_x = -frictionBoundary;
  } else {
    friction_x = frictionBoundary;
  }
  if (dyh_filt < frictionRange && dyh_filt > -frictionRange) {
    friction_y = - slope * dyh_filt;
  } else if (dyh_filt >= frictionRange) {
    friction_y = -frictionBoundary;
  } else {
    friction_y = frictionBoundary;
  }
  Fx += friction_x;
  Fy += friction_y;

  // Serial.print(" Friction x: ");
  // Serial.print(friction_x, 5);
  // Serial.print(",");
  // Serial.print("Friction y: ");
  // Serial.println(friction_y, 5);
  //////////////////////////////////////////Friction//////////////////////////////////////////////////////
  // //Calculate Torque Output from each motor
  T1 = rp / rs * (d1x3 * Fx + d1y3 * Fy);
  T5 = rp / rs * (d5x3 * Fx + d5y3 * Fy);
}

void resetForce(void) {
  T1 = 0;
  T5 = 0;
}


void getDynamics2(){
// Calculate x velocity with loop time estimation
  dxh = (xh - xh_prev) / dt;
  // Calculate x acceleration with loop time estimation
  axh = (dxh - dxh_prev) / dt;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
  // Calculate filtered acceleration
  axh_filt = 0.9 * axh + 0.1 * axh_prev;

  // Record the position and velocity
  xh_prev = xh;
  dxh_prev = dxh;
  dxh_filt_prev = dxh_filt;
  axh_prev = axh;
  axh_filt_prev = axh_filt;

  // Calculate y velocity with loop time estimation
  dyh = (yh - yh_prev) / dt;
  // Calculate y acceleration with loop time estimation
  ayh = (dyh - dyh_prev) / dt;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dyh_filt = .9*dyh + 0.1*dyh_prev; 
  // Calculate filtered acceleration
  ayh_filt = 0.9 * ayh + 0.1 * ayh_prev;
  // Record the position and velocity
  yh_prev = yh;
  dyh_prev = dyh;
  dyh_filt_prev = dyh_filt;
  ayh_prev = ayh;
  ayh_filt_prev = ayh_filt; 

// === Compute Spring Force from User Handle ===
  double FSpring_x = km * (xh - xProxy);
  double FSpring_y = km * (yh - yProxy);
  // === Damping Force (based on proxy velocity) ===
  double FDamping_x = bm * v_xProxy;
  double FDamping_y = bm * v_yProxy;
  // === Net Force on Virtual Object ===
  double ObjForce_x = FSpring_x - FDamping_x;
  double ObjForce_y = FSpring_y - FDamping_y;
  // === Compute Acceleration ===
  a_xProxy = ObjForce_x / mPuck;
  a_yProxy = ObjForce_y / mPuck;
  // === Velocity Update: Trapezoidal Integration ===
  v_xProxy += 0.5 * (last_a_xProxy + a_xProxy) * dt;
  v_yProxy += 0.5 * (last_a_yProxy + a_yProxy) * dt;
  // === Position Update: Trapezoidal Integration ===
  xProxy += 0.5 * (last_v_xProxy + v_xProxy) * dt;
  yProxy += 0.5 * (last_v_yProxy + v_yProxy) * dt;
  // === Optional: Limit Proxy Drift From Hand ===
  double max_dx = maxOffset;
  double max_dy = maxOffset;
  if (xProxy > xh + max_dx) xProxy = xh + max_dx;
  if (xProxy < xh - max_dx) xProxy = xh - max_dx;
  if (yProxy > yh + max_dy) yProxy = yh + max_dy;
  if (yProxy < yh - max_dy) yProxy = yh - max_dy;

  last_v_xProxy = v_xProxy;
  last_v_yProxy = v_yProxy;
  last_a_xProxy = a_xProxy;
  last_a_yProxy = a_yProxy;
  // === Output Force Feedback to Motors ===
  Fx = FSpring_x;
  Fy = FSpring_x;

  // Serial.print("Inertia x: ");
  // Serial.print(Fx, 5);
  // Serial.print(",");
  // Serial.print("Inertia y: ");
  // Serial.println(Fy, 5);

  //Calculate Torque Output from each motor
  T1 = rp / rs * (d1x3 * Fx + d1y3 * Fy);
  T5 = rp / rs * (d5x3 * Fx + d5y3 * Fy);
}


  /***************************************************************************
   @Function: setDutyCycles
   @Arguments: none
   @Returns: none
   @Description:
 ***************************************************************************/
void setDutyCycles(void) {
    // Compute the duty cycle required to generate T1 (torque at the motor pulley)

  duty1 = sqrt(abs(T1)/0.03);
  duty2 = sqrt(abs(T5)/0.03);
  // Make sure the duty cycle is between 0 and 100%
  if (duty1 > 1) {            
    duty1 = 1;
  } else if (duty1 < 0) { 
    duty1 = 0;
  }  
  if (duty2 > 1) {            
    duty2 = 1;
  } else if (duty2 < 0) { 
    duty2 = 0;
  }  
  output1 = (int)(duty1* 255);   // convert duty cycle to output signal
  output2 = (int)(duty2* 255); 
  analogWrite(pwmPin,output1);  // output the signal
  analogWrite(pwmPin2,output2);
}

void setDirections(void) {
  if (T1 > 0) { 
      digitalWrite(dirPin, HIGH);
    } else {
      digitalWrite(dirPin, LOW);
    }
  if (T5 > 0) { 
      digitalWrite(dirPin2, HIGH);
    } else {
      digitalWrite(dirPin2, LOW);
    }
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void readButton(void){
  // button.loop();  
  // int rawState = button.getState();
  // buttonState = (rawState == LOW) ? 1 : 0; 
  buttonState = digitalRead(buttonPin); 
}

void sendValues(void){
  Serial.print(xh, 5); 
  Serial.print(",");
  Serial.print(yh, 5);
  Serial.print(",");
  Serial.print(buttonState);
  Serial.print(",");
  Serial.print(dxh_filt, 5);
  Serial.print(",");
  Serial.println(dyh_filt, 5);
  
  // Serial.println(buttonState);
}

/*--------------------------------- ISRs ----------------------------------*/


/*--------------------------------- Tests ---------------------------------*/

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
