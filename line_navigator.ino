/*
===============================================================================
OSOYOO MECANUM LINE NAVIGATOR
===============================================================================

Hardware:
  - OSOYOO Mecanum Wheel Robotic Car Kit V2.1
  - Arduino Mega2560
  - OSOYOO Model Y Motor Driver
  - 4 DC Gear Motors
  - 4 Mecanum Wheels
  - 5-Channel Line Tracking Sensor
  - 2 IR Obstacle Sensors

Features:
  - PID line following
  - Sharp curve handling
  - Intersection detection
  - Configurable navigation strategy
  - Line recovery
  - Obstacle detection
  - Mecanum lateral movement
  - Intersection memory
  - Serial diagnostics

Unused hardware:
  - Servo motor
  - Ultrasonic sensor
  - Wi-Fi module
  - Bluetooth module

Line sensors:
  S1 = A4
  S2 = A3
  S3 = A2
  S4 = A1
  S5 = A0

Obstacle sensors:
  Left  = D3
  Right = D2

===============================================================================
*/

// ============================================================================
// MOTOR PINS
// ============================================================================

#define speedPinR             9
#define RightMotorDirPin1   22
#define RightMotorDirPin2   24

#define LeftMotorDirPin1    26
#define LeftMotorDirPin2    28
#define speedPinL            10

#define speedPinRB           11
#define RightMotorDirPin1B    5
#define RightMotorDirPin2B    6

#define LeftMotorDirPin1B     7
#define LeftMotorDirPin2B     8
#define speedPinLB            12


// ============================================================================
// LINE SENSOR PINS
// ============================================================================

#define sensor1 A4
#define sensor2 A3
#define sensor3 A2
#define sensor4 A1
#define sensor5 A0


// ============================================================================
// OBSTACLE SENSOR PINS
// ============================================================================

#define IR_LEFT   3
#define IR_RIGHT  2


// ============================================================================
// MOTION PARAMETERS
// ============================================================================

#define BASE_SPEED       70
#define MAX_SPEED        90
#define MIN_SPEED        35

#define CURVE_SPEED      70
#define SEARCH_SPEED     40
#define OBSTACLE_SPEED   38
#define SHIFT_SPEED      55


// ============================================================================
// PID PARAMETERS
// ============================================================================

float Kp = 0.30;
float Ki = 0.00;
float Kd = 0.075;

float integral = 0.0;

const float INTEGRAL_LIMIT = 4000.0;

int previousError = 0;

unsigned long previousPIDTime = 0;


// ============================================================================
// LINE POSITION WEIGHTS
// ============================================================================

const int WEIGHT_1 = -200;
const int WEIGHT_2 = -100;
const int WEIGHT_3 =    0;
const int WEIGHT_4 =  100;
const int WEIGHT_5 =  200;


// ============================================================================
// ROBOT STATES
// ============================================================================

enum RobotState
{
  FOLLOW_LINE,

  SHARP_LEFT,
  SHARP_RIGHT,

  INTERSECTION,

  OBSTACLE_LEFT,
  OBSTACLE_RIGHT,
  OBSTACLE_BOTH,

  ESCAPE_LEFT,
  ESCAPE_RIGHT,

  SEARCH_LINE,

  STOPPED
};

RobotState state = FOLLOW_LINE;


// ============================================================================
// LINE TRACKING MEMORY
// ============================================================================

int lastError = 0;

int lastDirection = 0;

unsigned long lastLineDetected = 0;


// ============================================================================
// INTERSECTION MEMORY
// ============================================================================

#define MAX_INTERSECTIONS 50

byte intersectionMemory[MAX_INTERSECTIONS];

int intersectionCount = 0;


// ============================================================================
// NAVIGATION STRATEGY
// ============================================================================

enum NavigationStrategy
{
  LEFT_HAND,
  RIGHT_HAND,
  STRAIGHT_FIRST
};

NavigationStrategy navigationStrategy = LEFT_HAND;


// ============================================================================
// LINE SENSOR DATA
// ============================================================================

struct LineData
{
  bool s1;
  bool s2;
  bool s3;
  bool s4;
  bool s5;

  int active;

  bool any;
  bool all;

  int error;
};


// ============================================================================
// MOTOR CONTROL
// ============================================================================

void FR_forward(int speed)
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);

  analogWrite(speedPinR, constrain(speed, 0, 255));
}


void FR_backward(int speed)
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);

  analogWrite(speedPinR, constrain(speed, 0, 255));
}


void FL_forward(int speed)
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);

  analogWrite(speedPinL, constrain(speed, 0, 255));
}


void FL_backward(int speed)
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);

  analogWrite(speedPinL, constrain(speed, 0, 255));
}


void RR_forward(int speed)
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);

  analogWrite(speedPinRB, constrain(speed, 0, 255));
}


void RR_backward(int speed)
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);

  analogWrite(speedPinRB, constrain(speed, 0, 255));
}


void RL_forward(int speed)
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);

  analogWrite(speedPinLB, constrain(speed, 0, 255));
}


void RL_backward(int speed)
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);

  analogWrite(speedPinLB, constrain(speed, 0, 255));
}


// ============================================================================
// STOP
// ============================================================================

void stopRobot()
{
  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinLB, 0);

  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);

  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);

  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, LOW);

  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, LOW);
}


// ============================================================================
// BASIC MOTION
// ============================================================================

void forward(int speed)
{
  FR_forward(speed);
  FL_forward(speed);
  RR_forward(speed);
  RL_forward(speed);
}


void backward(int speed)
{
  FR_backward(speed);
  FL_backward(speed);
  RR_backward(speed);
  RL_backward(speed);
}


void rotateLeft(int speed)
{
  FR_forward(speed);
  FL_backward(speed);

  RR_forward(speed);
  RL_backward(speed);
}


void rotateRight(int speed)
{
  FR_backward(speed);
  FL_forward(speed);

  RR_backward(speed);
  RL_forward(speed);
}


// ============================================================================
// MECANUM LATERAL MOTION
// ============================================================================

void shiftLeft(int speed)
{
  FR_backward(speed);
  FL_forward(speed);

  RR_forward(speed);
  RL_backward(speed);
}


void shiftRight(int speed)
{
  FR_forward(speed);
  FL_backward(speed);

  RR_backward(speed);
  RL_forward(speed);
}


// ============================================================================
// LINE SENSOR
// ============================================================================

bool readLineSensor(int pin)
{
  return !digitalRead(pin);
}


// ============================================================================
// LINE SENSOR PROCESSING
// ============================================================================

LineData readLine()
{
  LineData data;

  data.s1 = readLineSensor(sensor1);
  data.s2 = readLineSensor(sensor2);
  data.s3 = readLineSensor(sensor3);
  data.s4 = readLineSensor(sensor4);
  data.s5 = readLineSensor(sensor5);

  data.active =
    data.s1 +
    data.s2 +
    data.s3 +
    data.s4 +
    data.s5;

  data.any = data.active > 0;
  data.all = data.active == 5;

  long weighted = 0;
  long total = 0;

  if (data.s1)
  {
    weighted += WEIGHT_1;
    total += 100;
  }

  if (data.s2)
  {
    weighted += WEIGHT_2;
    total += 100;
  }

  if (data.s3)
  {
    weighted += WEIGHT_3;
    total += 100;
  }

  if (data.s4)
  {
    weighted += WEIGHT_4;
    total += 100;
  }

  if (data.s5)
  {
    weighted += WEIGHT_5;
    total += 100;
  }

  if (total > 0)
  {
    data.error = weighted / total;

    lastError = data.error;
    lastLineDetected = millis();

    if (data.error < -15)
      lastDirection = -1;
    else if (data.error > 15)
      lastDirection = 1;
  }
  else
  {
    data.error = lastError;
  }

  return data;
}


// ============================================================================
// OBSTACLE SENSORS
// ============================================================================

bool obstacleLeft()
{
  return digitalRead(IR_LEFT) == LOW;
}


bool obstacleRight()
{
  return digitalRead(IR_RIGHT) == LOW;
}


// ============================================================================
// SHARP CURVE DETECTION
// ============================================================================

bool sharpLeft(const LineData &line)
{
  if (line.s1 &&
      !line.s3 &&
      !line.s4 &&
      !line.s5)
  {
    return true;
  }

  if (line.s1 &&
      line.s2 &&
      !line.s3)
  {
    return true;
  }

  return false;
}


bool sharpRight(const LineData &line)
{
  if (line.s5 &&
      !line.s3 &&
      !line.s2 &&
      !line.s1)
  {
    return true;
  }

  if (line.s5 &&
      line.s4 &&
      !line.s3)
  {
    return true;
  }

  return false;
}


// ============================================================================
// INTERSECTION DETECTION
// ============================================================================

bool isIntersection(const LineData &line)
{
  if (line.all)
    return true;

  if (line.active >= 4)
    return true;

  if (line.s1 && line.s5)
    return true;

  return false;
}


// ============================================================================
// LINE FOLLOWING
// ============================================================================

void followLine(const LineData &line)
{
  if (!line.any)
  {
    state = SEARCH_LINE;
    return;
  }

  if (isIntersection(line))
  {
    state = INTERSECTION;
    return;
  }

  if (sharpLeft(line))
  {
    state = SHARP_LEFT;
    return;
  }

  if (sharpRight(line))
  {
    state = SHARP_RIGHT;
    return;
  }

  unsigned long now = millis();

  float dt =
    (now - previousPIDTime) / 1000.0;

  if (dt <= 0)
    dt = 0.008;

  previousPIDTime = now;

  float error = line.error;

  integral += error * dt;

  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  float derivative =
    (error - previousError) / dt;

  previousError = error;

  float correction =
    Kp * error +
    Ki * integral +
    Kd * derivative;

  correction =
    constrain(correction, -55, 55);

  int absoluteError = abs((int)error);

  int baseSpeed = BASE_SPEED;

  if (absoluteError > 150)
  {
    baseSpeed = CURVE_SPEED;
  }
  else if (absoluteError > 100)
  {
    baseSpeed = 52;
  }
  else if (absoluteError > 60)
  {
    baseSpeed = 58;
  }

  if (obstacleLeft() || obstacleRight())
  {
    baseSpeed =
      min(baseSpeed, OBSTACLE_SPEED);
  }

  int leftSpeed =
    baseSpeed + correction;

  int rightSpeed =
    baseSpeed - correction;

  leftSpeed =
    constrain(leftSpeed, MIN_SPEED, MAX_SPEED);

  rightSpeed =
    constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  FL_forward(leftSpeed);
  RL_forward(leftSpeed);

  FR_forward(rightSpeed);
  RR_forward(rightSpeed);
}


// ============================================================================
// SHARP LEFT TURN
// ============================================================================

void executeSharpLeft()
{
  unsigned long start = millis();

  while (millis() - start < 280)
  {
    LineData line = readLine();

    if (line.s3 ||
        (line.s2 && line.s3))
    {
      stopRobot();
      delay(30);

      state = FOLLOW_LINE;

      return;
    }

    rotateLeft(CURVE_SPEED);

    delay(5);
  }

  state = SEARCH_LINE;
}


// ============================================================================
// SHARP RIGHT TURN
// ============================================================================

void executeSharpRight()
{
  unsigned long start = millis();

  while (millis() - start < 280)
  {
    LineData line = readLine();

    if (line.s3 ||
        (line.s3 && line.s4))
    {
      stopRobot();
      delay(30);

      state = FOLLOW_LINE;

      return;
    }

    rotateRight(CURVE_SPEED);

    delay(5);
  }

  state = SEARCH_LINE;
}


// ============================================================================
// INTERSECTION DECISION
// ============================================================================

byte chooseDirection()
{
  switch (navigationStrategy)
  {
    case LEFT_HAND:
      return 1;

    case RIGHT_HAND:
      return 3;

    case STRAIGHT_FIRST:
      return 2;
  }

  return 1;
}


// ============================================================================
// INTERSECTION HANDLING
// ============================================================================

void executeIntersection()
{
  stopRobot();

  delay(80);

  int node = intersectionCount;

  if (intersectionCount < MAX_INTERSECTIONS)
  {
    intersectionMemory[intersectionCount] = 0;
    intersectionCount++;
  }

  byte direction =
    chooseDirection();

  Serial.print(F("INTERSECTION #"));
  Serial.print(node);

  Serial.print(F(" DECISION: "));

  if (direction == 1)
  {
    Serial.println(F("LEFT"));

    forward(CURVE_SPEED);
    delay(100);

    unsigned long start = millis();

    while (millis() - start < 550)
    {
      rotateLeft(CURVE_SPEED);

      LineData line = readLine();

      if (line.s3 || line.s2)
      {
        stopRobot();
        delay(40);

        if (node < MAX_INTERSECTIONS)
          intersectionMemory[node] = 1;

        state = FOLLOW_LINE;

        return;
      }

      delay(5);
    }

    if (node < MAX_INTERSECTIONS)
      intersectionMemory[node] = 4;

    state = SEARCH_LINE;

    return;
  }

  if (direction == 3)
  {
    Serial.println(F("RIGHT"));

    forward(CURVE_SPEED);
    delay(100);

    unsigned long start = millis();

    while (millis() - start < 550)
    {
      rotateRight(CURVE_SPEED);

      LineData line = readLine();

      if (line.s3 || line.s4)
      {
        stopRobot();
        delay(40);

        if (node < MAX_INTERSECTIONS)
          intersectionMemory[node] = 3;

        state = FOLLOW_LINE;

        return;
      }

      delay(5);
    }

    if (node < MAX_INTERSECTIONS)
      intersectionMemory[node] = 4;

    state = SEARCH_LINE;

    return;
  }

  if (direction == 2)
  {
    Serial.println(F("STRAIGHT"));

    forward(CURVE_SPEED);
    delay(180);

    if (node < MAX_INTERSECTIONS)
      intersectionMemory[node] = 2;

    state = FOLLOW_LINE;

    return;
  }
}


// ============================================================================
// OBSTACLE ANALYSIS
// ============================================================================

void analyzeObstacle()
{
  bool left = obstacleLeft();
  bool right = obstacleRight();

  if (!left && !right)
  {
    state = FOLLOW_LINE;
    return;
  }

  if (left && right)
  {
    state = OBSTACLE_BOTH;
    return;
  }

  if (left)
  {
    state = OBSTACLE_LEFT;
    return;
  }

  state = OBSTACLE_RIGHT;
}


// ============================================================================
// OBSTACLE HANDLING
// ============================================================================

void handleObstacle()
{
  bool left = obstacleLeft();
  bool right = obstacleRight();

  if (!left && !right)
  {
    state = FOLLOW_LINE;
    return;
  }

  if (left && right)
  {
    stopRobot();

    delay(100);

    left = obstacleLeft();
    right = obstacleRight();

    if (left && right)
    {
      if (lastDirection < 0)
        state = ESCAPE_RIGHT;
      else
        state = ESCAPE_LEFT;

      return;
    }

    state = FOLLOW_LINE;

    return;
  }

  if (left)
  {
    state = ESCAPE_RIGHT;
    return;
  }

  state = ESCAPE_LEFT;
}


// ============================================================================
// OBSTACLE ESCAPE - RIGHT
// ============================================================================

void executeEscapeRight()
{
  Serial.println(F("OBSTACLE -> SHIFT RIGHT"));

  stopRobot();

  delay(50);

  unsigned long start = millis();

  while (millis() - start < 350)
  {
    shiftRight(SHIFT_SPEED);
    delay(5);
  }

  stopRobot();

  delay(80);

  state = SEARCH_LINE;
}


// ============================================================================
// OBSTACLE ESCAPE - LEFT
// ============================================================================

void executeEscapeLeft()
{
  Serial.println(F("OBSTACLE -> SHIFT LEFT"));

  stopRobot();

  delay(50);

  unsigned long start = millis();

  while (millis() - start < 350)
  {
    shiftLeft(SHIFT_SPEED);
    delay(5);
  }

  stopRobot();

  delay(80);

  state = SEARCH_LINE;
}


// ============================================================================
// LINE SEARCH
// ============================================================================

void searchLine()
{
  Serial.println(F("SEARCHING FOR LINE"));

  unsigned long start = millis();

  const unsigned long TIMEOUT = 1800;

  while (millis() - start < TIMEOUT)
  {
    LineData line = readLine();

    if (line.any)
    {
      stopRobot();

      delay(40);

      state = FOLLOW_LINE;

      return;
    }

    if (lastDirection < 0)
    {
      rotateLeft(SEARCH_SPEED);
    }
    else if (lastDirection > 0)
    {
      rotateRight(SEARCH_SPEED);
    }
    else
    {
      rotateLeft(SEARCH_SPEED);
    }

    delay(5);
  }

  stopRobot();

  if (intersectionCount < MAX_INTERSECTIONS)
  {
    intersectionMemory[intersectionCount] = 4;
  }

  Serial.println(F("LINE NOT FOUND"));
  Serial.println(F("POSSIBLE DEAD END"));

  state = STOPPED;
}


// ============================================================================
// SERIAL DIAGNOSTICS
// ============================================================================

void printDebug(const LineData &line)
{
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 250)
    return;

  lastPrint = millis();

  Serial.print(F("LINE: "));

  Serial.print(line.s1);
  Serial.print(' ');

  Serial.print(line.s2);
  Serial.print(' ');

  Serial.print(line.s3);
  Serial.print(' ');

  Serial.print(line.s4);
  Serial.print(' ');

  Serial.print(line.s5);

  Serial.print(F(" | ERROR="));
  Serial.print(line.error);

  Serial.print(F(" | IR="));

  Serial.print(obstacleLeft());

  Serial.print('/');

  Serial.print(obstacleRight());

  Serial.print(F(" | INTERSECTIONS="));
  Serial.print(intersectionCount);

  Serial.print(F(" | STATE="));

  switch (state)
  {
    case FOLLOW_LINE:
      Serial.println(F("FOLLOW"));
      break;

    case SHARP_LEFT:
      Serial.println(F("SHARP_LEFT"));
      break;

    case SHARP_RIGHT:
      Serial.println(F("SHARP_RIGHT"));
      break;

    case INTERSECTION:
      Serial.println(F("INTERSECTION"));
      break;

    case OBSTACLE_LEFT:
      Serial.println(F("OBSTACLE_LEFT"));
      break;

    case OBSTACLE_RIGHT:
      Serial.println(F("OBSTACLE_RIGHT"));
      break;

    case OBSTACLE_BOTH:
      Serial.println(F("OBSTACLE_BOTH"));
      break;

    case ESCAPE_LEFT:
      Serial.println(F("ESCAPE_LEFT"));
      break;

    case ESCAPE_RIGHT:
      Serial.println(F("ESCAPE_RIGHT"));
      break;

    case SEARCH_LINE:
      Serial.println(F("SEARCH"));
      break;

    case STOPPED:
      Serial.println(F("STOPPED"));
      break;
  }
}


// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

void initGPIO()
{
  pinMode(speedPinR, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(speedPinRB, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);

  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  stopRobot();
}


// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);

  initGPIO();

  delay(500);

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F(" OSOYOO MECANUM LINE NAVIGATOR"));
  Serial.println(F("=========================================="));

  Serial.println(F("LINE: A4 A3 A2 A1 A0"));
  Serial.println(F("IR LEFT : D3"));
  Serial.println(F("IR RIGHT: D2"));

  Serial.println(F("PID: ENABLED"));
  Serial.println(F("INTERSECTIONS: ENABLED"));
  Serial.println(F("OBSTACLES: ENABLED"));
  Serial.println(F("MECANUM SHIFT: ENABLED"));

  Serial.println(F("=========================================="));

  lastLineDetected = millis();
  previousPIDTime = millis();

  delay(1000);
}


// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  LineData line = readLine();

  bool leftObstacle =
    obstacleLeft();

  bool rightObstacle =
    obstacleRight();

  if ((leftObstacle || rightObstacle) &&
      state == FOLLOW_LINE)
  {
    analyzeObstacle();
  }

  switch (state)
  {
    case FOLLOW_LINE:
      followLine(line);
      break;

    case SHARP_LEFT:
      executeSharpLeft();
      break;

    case SHARP_RIGHT:
      executeSharpRight();
      break;

    case INTERSECTION:
      executeIntersection();
      break;

    case OBSTACLE_LEFT:
      handleObstacle();
      break;

    case OBSTACLE_RIGHT:
      handleObstacle();
      break;

    case OBSTACLE_BOTH:
      handleObstacle();
      break;

    case ESCAPE_LEFT:
      executeEscapeLeft();
      break;

    case ESCAPE_RIGHT:
      executeEscapeRight();
      break;

    case SEARCH_LINE:
      searchLine();
      break;

    case STOPPED:
      stopRobot();
      break;
  }

  printDebug(line);

  delay(1);
}
