#include <EEPROM.h>
#include <SPI.h>
#include <PID_v1.h>
#include <AS5048A.h>
#include <math.h>
#include <stdbool.h>

#define SCARA_L1 150
#define SCARA_L2 150
#define SCARA_D  300
#define CENTER_X 150
#define CENTER_Y 75
#define CIRCLE_RADIUS 40
#define TIME_TO_CIRCLE  10000000
//#define TIME_TO_CIRCLE  5000000

// in degrees
#define MIN_ELBOW_ANGLE 18

void setup();
void loop();
void calcTarget();
void stop();
long normalizeInput(long absPosition, int target);
int readPositionA();
int readPositionB();
void pwmOut(int out, int pin1, int pin2);
void clearPositions();
void turnManually(int valueA, int valueB);
void processLine();
void printPos();
void help();
bool closeEnough(double value, double target);
double pow2(double x);
double capDegrees(double d);
int degreesToEncoder(double d);
int radiansToEncoder(double r);
double encoderToDegrees(int e);
double encoderToRadians(int e);
double sss(double a, double b, double c);
double sas(double aR, double b, double c);
void syncReverseShoulders(double x, double y, double *alphaD, double *betaD);
void syncReverseElbows(double x, double y, double *alphaD, double *betaD);
void syncForwardElbows(double alphaD, double betaD, double* x, double* y);
void writePIDToEEPROM();
void writeZeroToEEPROM();
void readFromEEPROM();

AS5048A angleSensorA(2);
AS5048A angleSensorB(6);

// these pins can all do PWM
#define OUT_A1 23
#define OUT_A2 22
#define OUT_B1 20
#define OUT_B2 21

#define LOG_COUNT 1000

// TODO: this should probably change to be able to record both motors'
// positions / deviations
int p = 0;
int pos[LOG_COUNT];
int skip = 0;

int start = 0;

//double kp=6,ki=32,kd=0.1;
double kp = 3, ki = 0, kd = 0;

double targetX = SCARA_D/2, targetY = SCARA_L1/2;

// these will be brought into sync with targetX & targetY in setup()
double inputA = 0, outputA = 0, targetA = 0;
double inputB = 0, outputB = 0, targetB = 0;

PID pidA(&inputA, &outputA, &targetA, kp, ki, kd, DIRECT);
PID pidB(&inputB, &outputB, &targetB, kp, ki, kd, DIRECT);

long absPositionA = 0, absPositionB = 0;
int zeroPositionA = 0, zeroPositionB = 0;

boolean enableRandomShapes = false, enableLogging = false, counting = false;
boolean enablePID = false;

void setup() {
  // get everything in sync using the initial targetX & targetY
  double alphaD, betaD;
  syncReverseElbows(targetX, targetY, &alphaD, &betaD);
  targetA = degreesToEncoder(alphaD);
  targetB = degreesToEncoder(betaD);

  pinMode(OUT_A1, OUTPUT);
  pinMode(OUT_A2, OUTPUT);
  pinMode(OUT_B1, OUTPUT);
  pinMode(OUT_B2, OUTPUT);
  Serial.begin(115200);
  help();

  // https://www.pjrc.com/teensy/td_pulse.html
  // NOTE: this should set all the PWM pins we use because they all use the
  // same timer
  analogWriteFrequency(22, 140625);
  analogWriteResolution(8);

  angleSensorA.init();
  angleSensorB.init();

  //Setup the pid
  pidA.SetMode(AUTOMATIC);
  pidA.SetSampleTime(1);
  pidA.SetOutputLimits(-255, 255);
  pidB.SetMode(AUTOMATIC);
  pidB.SetSampleTime(1);
  pidB.SetOutputLimits(-255, 255);

  readFromEEPROM();
}

void calcTarget() {
  double phase;
  double angle;
  double alpha;
  double beta;

  // TODO: at first the position will be way off and we're not taking the
  // time it will take to get there into account, but after a few times going
  // around and around it should even out.
  phase = (double)((micros() - start) % TIME_TO_CIRCLE) / TIME_TO_CIRCLE;
  angle = M_PI * 2 * phase;
  targetX = cos(angle)*CIRCLE_RADIUS + CENTER_X;
  targetY = sin(angle)*CIRCLE_RADIUS + CENTER_Y;
  // TODO: work in radians or straight in the encoder values rather?
  syncReverseElbows(targetX, targetY, &alpha, &beta);
  // TODO: validate here
  targetA = degreesToEncoder(alpha);
  targetB = degreesToEncoder(beta);
  //Serial.println(phase);
}

void stop() {
  pwmOut(0, OUT_A1, OUT_A2);
  pwmOut(0, OUT_B1, OUT_B2);
  if (enablePID) {
    Serial.println("stop");
  }
  enablePID = false;
}

boolean inDangerZone() {
  Serial.print("A: ");
  Serial.println(absPositionA);
  Serial.print("B: ");
  Serial.println(absPositionB);

  if ((absPositionA > degreesToEncoder(180 - MIN_ELBOW_ANGLE)) || (absPositionA < degreesToEncoder(90))) {
    Serial.println("A is in dangerzone.");

    Serial.print(absPositionA);
    Serial.print(" (");
    Serial.print(encoderToDegrees(absPositionA));
    Serial.print(")");
    Serial.print(" > ");
    Serial.print(degreesToEncoder(180 - MIN_ELBOW_ANGLE));
    Serial.print(" || ");
    Serial.print(absPositionA);
    Serial.print(" < ");
    Serial.println(degreesToEncoder(90));

    return true;
  }

  if ((absPositionB > degreesToEncoder(270)) || (absPositionB < degreesToEncoder(180 + MIN_ELBOW_ANGLE))) {
    Serial.println("B is in dangerzone.");

    Serial.print(absPositionB);
    Serial.print(" (");
    Serial.print(encoderToDegrees(absPositionB));
    Serial.print(")");
    Serial.print(" > ");
    Serial.print(degreesToEncoder(270));
    Serial.print(" || ");
    Serial.print(absPositionB);
    Serial.print(" < ");
    Serial.println(degreesToEncoder(180 + MIN_ELBOW_ANGLE));

    return true;
  }

  return false;
}

void loop() {
  if (angleSensorA.error()) {
    Serial.print("Sensor A Errors: ");
    Serial.println(angleSensorA.getErrors());
  }
  if (angleSensorB.error()) {
    Serial.print("Sensor B Errors: ");
    Serial.println(angleSensorB.getErrors());
  }

  // read these even when PID control is disabled because we might still want
  // to log it
  absPositionA = readPositionA();
  absPositionB = readPositionB();

  // have this ready in case we want to log it
  inputA = normalizeInput(absPositionA, targetA);
  inputB = normalizeInput(absPositionB, targetB);

  if (Serial.available()) {
    processLine(); // it may induce a glitch to move motion, so use it sparingly
  }

  if (enableLogging && millis() % 10000 == 0) {
    printPos();
  }

  if (enablePID) {
    // short circuit if we're in the danger zone for now
    if (inDangerZone()) {
      Serial.println("Danger!");
      stop();
      return;
    }

    // calculate the new target before we compute the PID
    if (enableRandomShapes) {
      calcTarget();
    }

    pidA.Compute();
    pidB.Compute();

    pwmOut(outputA, OUT_A1, OUT_A2);
    pwmOut(outputB, OUT_B1, OUT_B2);
  }

  // store current position(s) so it can be logged
  // TODO: do the same for inputB
  if (counting && (skip++ % 5) == 0) {
    pos[p] = inputA;
    if (p < LOG_COUNT-1) {
      p++;
    } else {
      counting = false;
    }
  }
}

int readPositionA() {
  long pos = ((long) angleSensorA.getRawRotation()) - zeroPositionA;
  if (pos < 0) {
    pos += 16384;
  }
  return pos;
}

int readPositionB() {
  long pos = ((long) angleSensorB.getRawRotation()) - zeroPositionB;
  if (pos < 0) {
    pos += 16384;
  }
  return pos;
}

long normalizeInput(long absPosition, int target) {
  // Turn it into a long that could be outside of the range of an int to make
  // it "wrap" the right way around rather than taking the long way when
  // crossing 0 or 16384
  if (absPosition - target >= 16384/2) {
    return absPosition - 16384;
  } else if (absPosition - target <= -16384/2) {
    return 16384 + absPosition;
  } else {
    return absPosition;
  }
}


void pwmOut(int out, int pin1, int pin2) {
  if (out > 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, out);
  } else {
    analogWrite(pin1, abs(out));
    analogWrite(pin2, 0);
  }
}

void clearPositions() {
  for(int i=0; i<p; i++)
    pos[i] = 0;
  p = 0;
}

void turnManually(int valueA, int valueB) {
  enablePID = false;
  pwmOut(valueA, OUT_A1, OUT_A2);
  pwmOut(valueB, OUT_B1, OUT_B2);
}

void processLine() {
 char cmd = Serial.read();
 double alpha, beta;

 if (cmd>'Z') cmd-=32;

 switch (cmd) {
  // A clockwise
  case '1':
    Serial.println("A clockwise");
    turnManually(255, 0);
    break;

  // A anti-clockwise
  case '2':
    Serial.println("A anti-clockwise");
    turnManually(-255, 0);
    break;

  // A/B stop
  case '3':
  case '6':
    Serial.println("A stop");
    stop();
    break;

  // B clockwise
  case '4':
    Serial.println("B clockwise");
    turnManually(0, 255);
    break;

  // B anti-clockwise
  case '5':
    Serial.println("B anti-clockwise");
    turnManually(0, -255);
    break;

  case 'P':
    kp = Serial.parseFloat();
    pidA.SetTunings(kp,ki,kd);
    break;

  case 'I':
    ki = Serial.parseFloat();
    pidA.SetTunings(kp,ki,kd);
    break;

  case 'D':
    kd = Serial.parseFloat();
    pidA.SetTunings(kp,ki,kd);
    break;

  case '?':
    printPos();
    break;

  // TODO: only enable once both X & Y are set?
  case 'X':
    targetX = Serial.parseFloat();

    syncReverseElbows(targetX, targetY,  &alpha, &beta);
    targetA = degreesToEncoder(alpha);
    targetB = degreesToEncoder(beta);

    counting = true;
    clearPositions();
    enablePID = true;
    break;

  case 'Y':
    targetY = Serial.parseFloat();

    syncReverseElbows(targetX, targetY,  &alpha, &beta);
    targetA = degreesToEncoder(alpha);
    targetB = degreesToEncoder(beta);

    counting = true;
    clearPositions();
    enablePID = true;
    break;

  // TODO: only enable once both A & B are set?
  case 'A':
    targetA = degreesToEncoder(Serial.parseFloat());
    syncForwardElbows(encoderToDegrees(targetA), encoderToDegrees(targetB), &targetX, &targetY);
    counting = true;
    clearPositions();
    enablePID = true;
    break;

  case 'B':
    targetB = degreesToEncoder(Serial.parseFloat());
    syncForwardElbows(encoderToDegrees(targetA), encoderToDegrees(targetB), &targetX, &targetY);
    counting = true;
    clearPositions();
    enablePID = true;
    break;

  case 'T':
    enableRandomShapes = !enableRandomShapes;
    if (enableRandomShapes) {
      Serial.println("starting...");
      start = micros();
      enablePID = true;
    } else {
      stop();
    }
    break;

  case 'L':
    enableLogging = !enableLogging;
    break;

  case 'Q':
    Serial.print("P=");
    Serial.print(kp);
    Serial.print(" I=");
    Serial.print(ki);
    Serial.print(" D=");
    Serial.println(kd);
    break;

  case 'H':
    help();
    break;

  case 'W':
    writePIDToEEPROM();
    break;

  case 'Z':
    Serial.println("Zero positions saved.");
    zeroPositionA = (int) angleSensorA.getRawRotation();
    zeroPositionB = (int) angleSensorB.getRawRotation();
    writeZeroToEEPROM();
    break;

  case 'S':
    for(int i=0; i<p; i++)
      Serial.println(pos[i]);
    break;
 }

 while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F("Position A=")); Serial.print(encoderToDegrees(absPositionA));
  Serial.print(F(" Goal A=")); Serial.print(capDegrees(encoderToDegrees(targetA)));
  Serial.print(F(" Input A=")); Serial.print(inputA);
  Serial.print(F(" Target A=")); Serial.print(targetA);
  Serial.print(F(" Output A=")); Serial.println(outputA);

  Serial.print(F("Position B=")); Serial.print(encoderToDegrees(absPositionB));
  Serial.print(F(" Goal B=")); Serial.print(capDegrees(encoderToDegrees(targetB)));
  Serial.print(F(" Input B=")); Serial.print(inputB);
  Serial.print(F(" Target B=")); Serial.print(targetB);
  Serial.print(F(" Output B=")); Serial.println(outputB);
}

void help() {
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("? prints out current input, output and target values"));
  Serial.println(F("X123 sets the target X position"));
  Serial.println(F("Y123 sets the target Y position"));
  Serial.println(F("A45 sets the target alpha angle"));
  Serial.println(F("B45 sets the target beta angle"));
  Serial.println(F("T will start drawing random shapes. T again will disable that"));
  Serial.println(F("Q will print out the current values of P, I and D parameters"));
  Serial.println(F("W will store current values of P, I and D parameters into EEPROM"));
  Serial.println(F("Z will store the current positions of the arms as zero into EEPROM"));
  Serial.println(F("H will print this help message again"));
  Serial.println(F("L will toggle on/off logging regular status every second\n"));
}

bool closeEnough(double value, double target) {
  return fabs(value - target) < 0.001;
}

double pow2(double x) {
  return pow(x, 2);
}

double capDegrees(double d) {
  while (d < 0) {
    d += 360;
  }
  while (d >= 360) {
    d -= 360;
  }
  return d;
}

int degreesToEncoder(double d) {
  return (int) (((double) d) / 360.0 * 16384.0);
}

int radiansToEncoder(double r) {
  return (int) (r / (PI*2) * 16384);
}


double encoderToDegrees(int e) {
  return ((double) e) / 16384.0 * 360.0;
}

double encoderToRadians(int e) {
  return ((double) e) / 16384.0 * PI*2;
}

// https://www.mathsisfun.com/algebra/trig-solving-sss-triangles.html
double sss(double a, double b, double c) {
  return acos((b*b + c*c - a*a) / (2*b*c));
}

// http://www.teacherschoice.com.au/maths_library/trigonometry/solve_trig_sas.htm
double sas(double aR, double b, double c) {
  return sqrt(b*b + c*c - 2*b*c*cos(aR));
}

void syncReverseShoulders(double x, double y, double *alphaD, double *betaD) {
  /*
  l1 is the length of the "upper" arms from the backboard/tower to an elbow
  joint. Or the vertical lines in a capital M.

  l2 is the length of the "fore" arms from the elbow to the end effector. Or
  the angled lines in a capital M.

  d is the length between the fixed points at the tower. Or the width of the M.

  A constraint is that the forearms will never form a straight line and the
  angle formed at the end effector on the inside will always be > 180 degrees.
  So it never starts to form a ^ shape. This avoids that singularity.

  {0, 0} is the position of the left tower assuming the arms always point away
  from you. Or the bottom-left corner of the M.

  {0, d} is the position of the right tower.

  {x, y} is the position of the end effector.

  c is the length of the line from {0, 0} to {x, y}

  e is the length of the line from {d, 0} to {x, y}

  alpha is the angle between the backboard and the left active arm

  beta is the angle between the backboard and the right active arm

  gamma is the angle formed between the lines c and l1 at {d, 0}

  epsilon is the angle formed between the lines e and l1 at {d, 0}

  delta is the angle formed between the lines c and d at {0, 0}

  psi is the angle formed between the lines e and d at {d, 0}
  */

  double c = sqrt(x*x + y*y);
  double e = sqrt(pow2(SCARA_D-x) + y*y);
  // NOTE: atan only defined for -90 to 90
  double deltaR = atan(y/x);
  // NOTE: acos only defined for 0 to 180
  double gammaR = sss(SCARA_L2, c, SCARA_L1);
  double epsilonR = sss(SCARA_L2, e, SCARA_L1);
  double psiR = atan(y/(SCARA_D-x));

  (*alphaD) = degrees(deltaR+gammaR);
  (*betaD) = 180 - degrees(epsilonR) - degrees(psiR);
}

void syncReverseElbows(double x, double y, double *alphaD, double *betaD) {
  /*
       #                   #
       ##                 ##
       # #               # #
       #  # l2       l2 #  #
       #   #           #   #
       #    #         #    #
       #     #       #     #
       #      #     #      #
       #       #   #       #
       #        # #        #
       #         #         #
    l1 #      (x, y)       # l1
       #                   #
       #                   #
       #                   #
       #                   #
       #                   #
       #                   #
       #                   #
       # (0, 0)            # (d, 0)

       *                   *
       *                   *
       *                   *
       *                   *
       #%%                 #%%
       ##%%alpha         ##%%beta
       #%#%              #%#%
       #%%#             #%%#
       #   #           #   #
       #    #         #    #
       #     #       #     #
       #gamma #     #      #
       #       #   #epsilon#
       #        # #        #
       #         #         #
       #        * *        #
       #       *   *       #
       #      *     *      #
       #    **       **    #
       #   * c      e  *   #
       #  *             *  #
       # *               * #
       #*        d        *#
       #*******************#

  l1 is the length of the "upper" arms from the backboard/tower to an elbow
  joint. Or the vertical lines in a capital M.

  l2 is the length of the "fore" arms from the elbow to the end effector. Or
  the angled lines in a capital M.

  d is the length between the fixed points at the tower. Or the width of the M.

  A constraint is that the forearms will never form a straight line and the
  angle formed at the end effector on the inside will always be > 180 degrees.
  So it never starts to form a ^ shape. This avoids that singularity.

  {0, 0} is the position of the left tower assuming the arms always point away
  from you. Or the bottom-left corner of the M.

  {0, d} is the position of the right tower.

  {x, y} is the position of the end effector.

  c is the length of the line from {0, 0} to {x, y}

  e is the length of the line from {d, 0} to {x, y}

  Imagine you extend both upper arms past the elbows. So the vertical lines in
  the M. Picture them extending way past those corners.

  alpha is the angle formed between this imaginary line and l2 (the angled line
  in the M or the forearm), going clockwise. This is the position of the "left"
  absolute angle position sensor assuming the towers are closest to you and the
  upper arms point away from you.

  beta is the same thing except for the "right" one.

  NOTE: alpha is always > 180 and beta is always < 180.

  gamma is the angle formed on the inside at the left elbow between l1 and l2
  with c opposite to it. (the one next to alpha)

  epsilon is the angle formed on the inside at the right elbow between l1 and
  l2 with e opposite to it. (the one next to beta)
  */

  double c = sqrt(x*x + y*y);
  double e = sqrt(pow2(SCARA_D-x) + y*y);

  double gammaR = sss(c, SCARA_L1, SCARA_L2);
  double epsilonR = sss(e, SCARA_L1, SCARA_L2);

  // TODO: rather stick to radians always
  (*alphaD) = 180-degrees(gammaR);
  (*betaD) = 180+degrees(epsilonR);
}

// TODO: radians rather
void syncForwardElbows(double alphaD, double betaD, double* x, double* y) {
  double c = sas(radians(180-alphaD), SCARA_L1, SCARA_L2);
  double e = sas(radians(betaD-180), SCARA_L1, SCARA_L2);
  double theta = sss(e, c, SCARA_D);
  (*x) = c * cos(theta);
  (*y) = c * sin(theta);
}


void writePIDToEEPROM() {
 // TODO
}

void writeZeroToEEPROM() {
  EEPROM.put(0, zeroPositionA);
  EEPROM.put(sizeof(zeroPositionA), zeroPositionB);
}

void readFromEEPROM() {
  EEPROM.get(0, zeroPositionA);
  EEPROM.get(sizeof(zeroPositionA), zeroPositionB);
  // TODO: read PID values too
}
