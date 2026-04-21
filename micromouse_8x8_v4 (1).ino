#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

/* ================= MOTOR PINS ================= */
#define ENA 25
#define IN1 27
#define IN2 14
#define ENB 26
#define IN3 12
#define IN4 13

/* ================= ENCODER PINS ================= */
#define ENC_A1 32
#define ENC_A2 33
#define ENC_B1 18
#define ENC_B2 19

/* ================= VL53L0X XSHUT PINS ================= */
#define XSHUT_FRONT 16
#define XSHUT_LEFT  17
#define XSHUT_RIGHT 4

volatile long countA = 0;
volatile long countB = 0;

void IRAM_ATTR encoderA_ISR(){ if (digitalRead(ENC_A2)) countA++; else countA--; }
void IRAM_ATTR encoderB_ISR(){ if (digitalRead(ENC_B2)) countB++; else countB--; }

/* ================= SENSORS ================= */
VL53L0X frontSensor;
VL53L0X leftSensor;
VL53L0X rightSensor;

/* ================= MPU ================= */
Adafruit_MPU6050 mpu;
float gz_bias = 0.0f;
float yaw = 0.0f;
unsigned long lastUs = 0;

/* ================= MOTOR SETTINGS ================= */
int baseA = 65;
int baseB = 55;

int SIGN = -1;
float Kp_yaw = 220.0f;
float Kd_rate = 25.0f;
int corrLimit = 70;

int wallThresholdFront = 100;
int wallThresholdSide  = 120;

long squareCounts = 240;
long turnCounts90 = 85;
int  turnPWM = 85;
unsigned long turnTimeoutMs = 2000;

/* ================= WALL CORRECTION - PD ================= */
float Kp_wall    = 1.2f;   // قلّلنا من 1.8 لتقليل الهز
float Kd_wall    = 0.8f;   // قلّلنا من 2.0
float centerTarget = 35.0f;
int wallCorrLimit  = 35;   // قلّلنا من 50
int dangerZone     = 20;
int dangerPush     = 60;   // قلّلنا من 80

int cachedLeft  = 999;
int cachedRight = 999;
int wallCorr = 0;
int prevWallError = 0;  // للـ derivative

long lastCheckTicks = 0;
int squareDelayMs = 300;

/* ================= CENTERING ================= */
bool enableCentering      = true;
bool enableFrontCentering = false;
int  centeringSpeed       = 50;
int  centerTolerance      = 3;
int  maxCenteringTime     = 1000;

/* =================================================================
 *  MAZE / FLOOD FILL  —  8 x 8
 * ================================================================= */
#define MAZE_SIZE 8

#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

#define N_WALL 1
#define E_WALL 2
#define S_WALL 4
#define W_WALL 8

int maze[MAZE_SIZE][MAZE_SIZE];
int flood[MAZE_SIZE][MAZE_SIZE];

int posX = 0;
int posY = 0;
int heading = NORTH;
bool goalReached = false;

/* ---- خلايا الهدف الأربع في المنتصف ---- */
const int GOAL_CELLS[4][2] = {{3,3},{3,4},{4,3},{4,4}};

bool isGoal(int x, int y){
  return (x == 3 || x == 4) && (y == 3 || y == 4);
}

/* ================= FLOOD FILL ================= */
bool inBounds(int x, int y){
  return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

bool isAccessible(int x, int y, int nx, int ny){
  if(!inBounds(nx, ny)) return false;
  if(nx == x && ny == y+1) return !(maze[x][y] & N_WALL);
  if(nx == x+1 && ny == y) return !(maze[x][y] & E_WALL);
  if(nx == x && ny == y-1) return !(maze[x][y] & S_WALL);
  if(nx == x-1 && ny == y) return !(maze[x][y] & W_WALL);
  return false;
}

bool isConsistent(int x, int y){
  int v = flood[x][y];
  if(v == 0) return true;
  int dirs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
  for(int i=0; i<4; i++){
    int nx = x + dirs[i][0];
    int ny = y + dirs[i][1];
    if(inBounds(nx,ny) && isAccessible(x,y,nx,ny))
      if(flood[nx][ny] == v - 1) return true;
  }
  return false;
}

void makeConsistent(int x, int y){
  int dirs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
  int m = 9999;
  for(int i=0; i<4; i++){
    int nx = x + dirs[i][0];
    int ny = y + dirs[i][1];
    if(inBounds(nx,ny) && isAccessible(x,y,nx,ny))
      if(flood[nx][ny] < m) m = flood[nx][ny];
  }
  flood[x][y] = m + 1;
}

void floodFill(){
  bool changed = true;
  while(changed){
    changed = false;
    for(int x=0; x<MAZE_SIZE; x++)
      for(int y=0; y<MAZE_SIZE; y++)
        if(!isConsistent(x,y)){
          makeConsistent(x,y);
          changed = true;
        }
  }
}

void initFlood(){
  for(int x=0; x<MAZE_SIZE; x++){
    for(int y=0; y<MAZE_SIZE; y++){
      int d = 9999;
      for(int i=0; i<4; i++){
        int dist = abs(x - GOAL_CELLS[i][0]) + abs(y - GOAL_CELLS[i][1]);
        if(dist < d) d = dist;
      }
      flood[x][y] = d;
    }
  }
  for(int i=0; i<4; i++)
    flood[GOAL_CELLS[i][0]][GOAL_CELLS[i][1]] = 0;
}

/* ================= WALL SETTING ================= */
void setWall(int x, int y, int dir){
  maze[x][y] |= dir;
  if(dir == N_WALL && y+1 < MAZE_SIZE) maze[x][y+1] |= S_WALL;
  if(dir == E_WALL && x+1 < MAZE_SIZE) maze[x+1][y] |= W_WALL;
  if(dir == S_WALL && y-1 >= 0)        maze[x][y-1] |= N_WALL;
  if(dir == W_WALL && x-1 >= 0)        maze[x-1][y] |= E_WALL;
}

/* ================= SENSE WALLS ================= */
void senseWalls(int x, int y, int h){
  int fD = readFrontDistance();
  int lD = readLeftDistance();
  int rD = readRightDistance();

  bool wallF = (fD < wallThresholdFront);
  bool wallL = (lD < wallThresholdSide);
  bool wallR = (rD < wallThresholdSide);

  if(wallF){
    if(h==NORTH) setWall(x,y,N_WALL);
    if(h==EAST)  setWall(x,y,E_WALL);
    if(h==SOUTH) setWall(x,y,S_WALL);
    if(h==WEST)  setWall(x,y,W_WALL);
  }
  if(wallL){
    if(h==NORTH) setWall(x,y,W_WALL);
    if(h==EAST)  setWall(x,y,N_WALL);
    if(h==SOUTH) setWall(x,y,E_WALL);
    if(h==WEST)  setWall(x,y,S_WALL);
  }
  if(wallR){
    if(h==NORTH) setWall(x,y,E_WALL);
    if(h==EAST)  setWall(x,y,S_WALL);
    if(h==SOUTH) setWall(x,y,W_WALL);
    if(h==WEST)  setWall(x,y,N_WALL);
  }

  Serial.print("SENSE ("); Serial.print(x); Serial.print(","); Serial.print(y);
  Serial.print(") h="); Serial.print(h);
  Serial.print(" F="); Serial.print(fD);
  Serial.print(" L="); Serial.print(lD);
  Serial.print(" R="); Serial.println(rD);
}

/* ================= UNKNOWN SCORE ================= */
int unknownScore(int x, int y){
  if(!inBounds(x,y)) return -1;
  int score = 0;
  if(!(maze[x][y] & N_WALL)) score++;
  if(!(maze[x][y] & E_WALL)) score++;
  if(!(maze[x][y] & S_WALL)) score++;
  if(!(maze[x][y] & W_WALL)) score++;
  return score;
}

/* ================= SELECT DIRECTION ================= */
int selectDirection(int x, int y, int h){
  int dirs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
  int bestDir = -1, bestFlood = 9999, bestUnknown = -1;
  int fallback = -1;

  int px = x - dirs[h][0];
  int py = y - dirs[h][1];

  for(int d=0; d<4; d++){
    int nx = x + dirs[d][0];
    int ny = y + dirs[d][1];

    if(!inBounds(nx,ny)) continue;
    if(!isAccessible(x,y,nx,ny)) continue;

    if(nx == px && ny == py){
      fallback = d;
      continue;
    }

    int fval = flood[nx][ny];
    int unk  = unknownScore(nx,ny);

    if(fval < bestFlood || (fval == bestFlood && unk > bestUnknown)){
      bestFlood   = fval;
      bestUnknown = unk;
      bestDir     = d;
    }
  }

  if(bestDir == -1) bestDir = fallback;
  return bestDir;
}

/* ================= PRINT FLOOD MAP ================= */
void printFlood(){
  Serial.println("--- FLOOD 8x8 ---");
  for(int y=MAZE_SIZE-1; y>=0; y--){
    for(int x=0; x<MAZE_SIZE; x++){
      if(x==posX && y==posY) Serial.print("[");
      else Serial.print(" ");
      if(flood[x][y] < 10) Serial.print(" ");
      Serial.print(flood[x][y]);
      if(x==posX && y==posY) Serial.print("]");
      else Serial.print(" ");
    }
    Serial.println();
  }
}

/* ================= IMU ================= */
float readGz(){
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  return g.gyro.z;
}

void calibrateGyro(){
  const int N2=800;
  float sum=0;
  for(int i=0;i<N2;i++){ sum += readGz(); delay(3); }
  gz_bias = sum / N2;
}

/* ================= TOF SETUP ================= */
void setupSensors(){
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_FRONT, HIGH); delay(10);
  frontSensor.init();
  frontSensor.setAddress(0x30);

  digitalWrite(XSHUT_LEFT, HIGH); delay(10);
  leftSensor.init();
  leftSensor.setAddress(0x31);

  digitalWrite(XSHUT_RIGHT, HIGH); delay(10);
  rightSensor.init();
  rightSensor.setAddress(0x32);

  frontSensor.setTimeout(250);
  leftSensor.setTimeout(250);
  rightSensor.setTimeout(250);
}

int readFrontDistance(){
  int d = frontSensor.readRangeSingleMillimeters();
  if(frontSensor.timeoutOccurred()) return 999;
  return d;
}

int readLeftDistance(){
  int d = leftSensor.readRangeSingleMillimeters();
  if(leftSensor.timeoutOccurred()) return 999;
  return d;
}

int readRightDistance(){
  int d = rightSensor.readRangeSingleMillimeters();
  if(rightSensor.timeoutOccurred()) return 999;
  return d;
}

/* ================= MOTOR CONTROL ================= */
void setForward(){
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void spinRight(){
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void spinLeft(){
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void resetCountersSafe(){
  detachInterrupt(digitalPinToInterrupt(ENC_A1));
  detachInterrupt(digitalPinToInterrupt(ENC_B1));
  countA = 0; countB = 0;
  delay(10);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);
}

/* ================= CENTERING ================= */
void centerBetweenWalls(){
  if(!enableCentering) return;

  int leftDist  = readLeftDistance();
  int rightDist = readRightDistance();

  bool hasLeft  = (leftDist  < wallThresholdSide);
  bool hasRight = (rightDist < wallThresholdSide);

  if(!hasLeft || !hasRight){
    Serial.println("Can't center - missing wall(s)");
    return;
  }

  Serial.print("Centering: L="); Serial.print(leftDist);
  Serial.print(" R="); Serial.println(rightDist);

  unsigned long startTime = millis();

  while(millis() - startTime < maxCenteringTime){
    leftDist  = readLeftDistance();
    rightDist = readRightDistance();
    int diff  = rightDist - leftDist;

    if(abs(diff) <= centerTolerance){
      stopMotors();
      Serial.println("Centered!");
      delay(50);
      return;
    }

    if(diff > centerTolerance){
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, centeringSpeed);
    analogWrite(ENB, centeringSpeed);
    delay(10);
  }

  stopMotors();
  Serial.println("Centering timeout");
  delay(50);
}

void centerInSquareFront(){
  if(!enableCentering || !enableFrontCentering) return;

  int frontDist = readFrontDistance();
  if(frontDist > wallThresholdFront){
    Serial.println("No front wall to center against");
    return;
  }

  int targetDist = 97;
  int diff = frontDist - targetDist;

  if(abs(diff) <= 5){ Serial.println("Already front-centered!"); return; }

  unsigned long startTime = millis();

  while(millis() - startTime < maxCenteringTime){
    frontDist = readFrontDistance();
    diff = frontDist - targetDist;

    if(abs(diff) <= 5){
      stopMotors();
      Serial.println("Front centered!");
      delay(50);
      return;
    }

    if(diff > 5){
      setForward();
    } else {
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, centeringSpeed);
    analogWrite(ENB, centeringSpeed);
    delay(10);
  }

  stopMotors();
  delay(50);
}

void fullCenter(){
  if(!enableCentering) return;
  Serial.println("=== CENTERING ===");
  centerBetweenWalls();
  delay(100);
  if(enableFrontCentering){
    centerInSquareFront();
    delay(100);
  }
  Serial.println("=== CENTERING DONE ===");
}

/* ================= TURNS ================= */
void turnRight90(){
  stopMotors(); delay(150);
  resetCountersSafe();
  spinRight();
  analogWrite(ENA, turnPWM); analogWrite(ENB, turnPWM);

  unsigned long start = millis();
  while(true){
    long ticksA = labs(countA);
    long ticksB = labs(countB);

    if(ticksA >= turnCounts90 && ticksB >= turnCounts90){
      for(int p=turnPWM; p>=0; p-=15){
        analogWrite(ENA,p); analogWrite(ENB,p); delay(10);
      }
      stopMotors(); delay(100);
      Serial.print("Turn R: A="); Serial.print(ticksA);
      Serial.print(" B="); Serial.println(ticksB);
      return;
    }

    if     (ticksA > ticksB + 5){ analogWrite(ENA, turnPWM*0.7); analogWrite(ENB, turnPWM); }
    else if(ticksB > ticksA + 5){ analogWrite(ENA, turnPWM);     analogWrite(ENB, turnPWM*0.7); }
    else                        { analogWrite(ENA, turnPWM);     analogWrite(ENB, turnPWM); }

    if(millis()-start > turnTimeoutMs){ stopMotors(); delay(100); return; }
    delay(2);
  }
}

void turnLeft90(){
  stopMotors(); delay(150);
  resetCountersSafe();
  spinLeft();
  analogWrite(ENA, turnPWM); analogWrite(ENB, turnPWM);

  unsigned long start = millis();
  while(true){
    long ticksA = labs(countA);
    long ticksB = labs(countB);

    if(ticksA >= turnCounts90 && ticksB >= turnCounts90){
      for(int p=turnPWM; p>=0; p-=15){
        analogWrite(ENA,p); analogWrite(ENB,p); delay(10);
      }
      stopMotors(); delay(100);
      Serial.print("Turn L: A="); Serial.print(ticksA);
      Serial.print(" B="); Serial.println(ticksB);
      return;
    }

    if     (ticksA > ticksB + 5){ analogWrite(ENA, turnPWM*0.7); analogWrite(ENB, turnPWM); }
    else if(ticksB > ticksA + 5){ analogWrite(ENA, turnPWM);     analogWrite(ENB, turnPWM*0.7); }
    else                        { analogWrite(ENA, turnPWM);     analogWrite(ENB, turnPWM); }

    if(millis()-start > turnTimeoutMs){ stopMotors(); delay(100); return; }
    delay(2);
  }
}

/* ================= STRAIGHTEN - بالجيرو + الحيطان ================= */
void straightenYaw(){
  Serial.print("Pre-straighten yaw="); Serial.println(yaw);

  // --- طريقة 1: استخدم الحيطان الجانبية إذا موجودة ---
  int lD = readLeftDistance();
  int rD = readRightDistance();
  bool hasL = (lD < wallThresholdSide);
  bool hasR = (rD < wallThresholdSide);

  if(hasL || hasR){
    // اتحرك شوي للأمام مع جيرو لحين يستوي
    unsigned long start = millis();
    lastUs = micros();

    while(millis() - start < 400){
      unsigned long now2 = micros();
      float dt2 = (now2 - lastUs) / 1e6f;
      lastUs = now2;

      float gz2 = readGz() - gz_bias;
      yaw += gz2 * dt2;

      if(abs(yaw) <= 0.015f) break;

      float err2 = 0.0f - yaw;
      int corr2 = (int)(SIGN * (Kp_yaw * err2 - Kd_rate * gz2));
      corr2 = constrain(corr2, -40, 40);

      int pwmA2 = constrain(30 - corr2, 0, 60);
      int pwmB2 = constrain(30 + corr2, 0, 60);

      setForward();
      analogWrite(ENA, pwmA2);
      analogWrite(ENB, pwmB2);
      delay(3);
    }
    stopMotors();
    delay(80);

  } else {
    // --- طريقة 2: بدون حيطان - صحح بدوران بطيء ---
    if(abs(yaw) < 0.03f){
      Serial.println("Yaw OK");
      return;
    }

    unsigned long start = millis();
    lastUs = micros();

    while(abs(yaw) > 0.018f && millis() - start < 500){
      unsigned long now2 = micros();
      float dt2 = (now2 - lastUs) / 1e6f;
      lastUs = now2;

      float gz2 = readGz() - gz_bias;
      yaw += gz2 * dt2;

      if(yaw > 0.018f){
        spinLeft();
      } else {
        spinRight();
      }
      analogWrite(ENA, 30);
      analogWrite(ENB, 30);
      delay(2);
    }
    stopMotors();
    delay(80);
  }

  Serial.print("Post-straighten yaw="); Serial.println(yaw);
}

/* ================= GRADUAL STOP مع جيرو أثناء التوقف ================= */
void gradualStop(){
  int steps = 8;
  for(int p = steps; p >= 0; p--){
    float fraction = (float)p / steps;
    int curA = (int)(baseA * fraction);
    int curB = (int)(baseB * fraction);

    // الجيرو يبقى شغّال أثناء التوقف التدريجي
    unsigned long now2 = micros();
    float dt2 = (now2 - lastUs) / 1e6f;
    lastUs = now2;
    float gz2 = readGz() - gz_bias;
    yaw += gz2 * dt2;

    float err2 = 0.0f - yaw;
    int corr2 = (int)(SIGN * Kp_yaw * err2);
    corr2 = constrain(corr2, -30, 30);

    int pwmA2 = constrain(curA - corr2, 0, 180);
    int pwmB2 = constrain(curB + corr2, 0, 180);

    setForward();
    analogWrite(ENA, pwmA2);
    analogWrite(ENB, pwmB2);
    delay(12);
  }
  stopMotors();
  delay(100);
}

void turnToFace(int targetDir){
  int diff = (targetDir - heading + 4) % 4;
  if(diff != 0){
    straightenYaw();   // ← صحّح الزاوية أولاً
    fullCenter();      // ← ثم اضبط الموقع
  }

  switch(diff){
    case 0: break;
    case 1: turnRight90(); Serial.println(">> TURN RIGHT"); break;
    case 2: turnRight90(); delay(50); turnRight90(); Serial.println(">> TURN 180"); break;
    case 3: turnLeft90();  Serial.println(">> TURN LEFT");  break;
  }
  heading = targetDir;
}

/* ================= UPDATE POSITION ================= */
void updatePosition(){
  switch(heading){
    case NORTH: posY++; break;
    case EAST:  posX++; break;
    case SOUTH: posY--; break;
    case WEST:  posX--; break;
  }
}

/* ================= RESUME DRIVING ================= */
void resumeDriving(){
  yaw = 0;
  lastUs = micros();
  resetCountersSafe();
  lastCheckTicks = 0;
  wallCorr = 0;
  prevWallError = 0;  // reset الـ derivative
  setForward();
  analogWrite(ENA, baseA);
  analogWrite(ENB, baseB);
}

/* ================= FLOOD DECISION ================= */
void doFloodDecision(){
  senseWalls(posX, posY, heading);
  floodFill();
  printFlood();

  if(flood[posX][posY] == 0 || isGoal(posX, posY)){
    goalReached = true;
    stopMotors();
    Serial.println("=== GOAL REACHED! ===");
    return;
  }

  int dir = selectDirection(posX, posY, heading);
  Serial.print("Select dir="); Serial.println(dir);

  if(dir == -1){
    Serial.println("No move! Turn around.");
    dir = (heading + 2) % 4;
  }

  if(dir != heading){
    stopMotors(); delay(50);
    turnToFace(dir);
  }
}

/* ================= SETUP ================= */
void setup(){
  Serial.begin(115200);

  pinMode(ENA,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(ENB,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  stopMotors();

  pinMode(ENC_A1,INPUT_PULLUP); pinMode(ENC_A2,INPUT_PULLUP);
  pinMode(ENC_B1,INPUT_PULLUP); pinMode(ENC_B2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);

  Wire.begin(21,22);
  setupSensors();

  if(!mpu.begin()){
    Serial.println("MPU6050 failed!");
    while(1){ delay(100); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(2000);
  Serial.println("Calibrating... keep still!");
  calibrateGyro();

  Serial.print("Front: "); Serial.print(readFrontDistance()); Serial.println(" mm");
  Serial.print("Left:  "); Serial.print(readLeftDistance());  Serial.println(" mm");
  Serial.print("Right: "); Serial.print(readRightDistance()); Serial.println(" mm");

  for(int x=0; x<MAZE_SIZE; x++)
    for(int y=0; y<MAZE_SIZE; y++)
      maze[x][y] = 0;

  for(int i=0; i<MAZE_SIZE; i++){
    maze[i][0]           |= S_WALL;
    maze[i][MAZE_SIZE-1] |= N_WALL;
    maze[0][i]           |= W_WALL;
    maze[MAZE_SIZE-1][i] |= E_WALL;
  }

  initFlood();

  posX = 0; posY = 0; heading = NORTH;

  doFloodDecision();
  if(goalReached){ while(1) delay(1000); }

  Serial.println("Starting in 2 seconds...");
  delay(2000);

  resumeDriving();
  Serial.println("=== GO ===");
}

/* ================= LOOP ================= */
void loop(){
  if(goalReached){ stopMotors(); return; }

  // === 1. GYRO STRAIGHT LINE ===
  unsigned long now = micros();
  float dt = (now - lastUs) / 1e6f;
  if(dt < 0.005f) return;
  lastUs = now;

  float gz = readGz() - gz_bias;
  yaw += gz * dt;

  float err = 0.0f - yaw;
  int gyroCorr = (int)(SIGN * (Kp_yaw * err - Kd_rate * gz));

  int corr = gyroCorr + wallCorr;
  corr = constrain(corr, -corrLimit, corrLimit);

  int pwmA = constrain(baseA - corr, 0, 180);
  int pwmB = constrain(baseB + corr, 0, 180);

  setForward();
  analogWrite(ENA, pwmA);
  analogWrite(ENB, pwmB);

  unsigned long nowMs = millis();

  // === 2. WALL CORRECTION - PD محسّن ===
  static unsigned long lastSideRead = 0;
  if(nowMs - lastSideRead > 30){   // كان 100ms → صار 30ms
    lastSideRead = nowMs;

    cachedLeft  = readLeftDistance();
    cachedRight = readRightDistance();

    bool wL = (cachedLeft  < wallThresholdSide);
    bool wR = (cachedRight < wallThresholdSide);

    int wallError = 0;

    if(wL && wR){
      // كلا الجانبين: اضبط على المنتصف
      wallError = cachedRight - cachedLeft;

    } else if(wR){
      // يمين فقط
      wallError = cachedRight - (int)centerTarget;
      // DANGER ZONE: قريب جداً من اليمين
      if(cachedRight < dangerZone){
        wallError = dangerPush;
        Serial.println("!! DANGER RIGHT !!");
      }

    } else if(wL){
      // يسار فقط
      wallError = (int)centerTarget - cachedLeft;
      wallError = -wallError;
      // DANGER ZONE: قريب جداً من اليسار
      if(cachedLeft < dangerZone){
        wallError = -dangerPush;
        Serial.println("!! DANGER LEFT !!");
      }

    } else {
      wallError = 0;
    }

    // PD: proportional + derivative
    int wallD = wallError - prevWallError;
    prevWallError = wallError;

    wallCorr = (int)(Kp_wall * (float)wallError + Kd_wall * (float)wallD);
    wallCorr = constrain(wallCorr, -wallCorrLimit, wallCorrLimit);

    // Debug
    Serial.print("WallCorr="); Serial.print(wallCorr);
    Serial.print(" L="); Serial.print(cachedLeft);
    Serial.print(" R="); Serial.println(cachedRight);
  }

  // === 3. FRONT EMERGENCY STOP ===
  static unsigned long lastFrontCheck = 0;
  if(nowMs - lastFrontCheck > 80){
    lastFrontCheck = nowMs;

    int frontDist = readFrontDistance();
    if(frontDist < 50){
      stopMotors(); delay(50);
      Serial.print("EMERGENCY F="); Serial.println(frontDist);

      senseWalls(posX, posY, heading);
      floodFill();

      int dir = selectDirection(posX, posY, heading);
      if(dir == -1) dir = (heading + 2) % 4;

      turnToFace(dir);
      resumeDriving();
      return;
    }
  }

  // === 4. SQUARE BOUNDARY: FLOOD FILL DECISION ===
  long currentTicks = (labs(countA) + labs(countB)) / 2;

  if(currentTicks - lastCheckTicks >= squareCounts){
    lastCheckTicks = currentTicks;

    gradualStop();     // ← توقف تدريجي بدل المفاجئ
    updatePosition();

    Serial.println("========================================");
    Serial.print(">> SQUARE! (");
    Serial.print(posX); Serial.print(","); Serial.print(posY); Serial.println(")");
    Serial.println("========================================");

    delay(squareDelayMs);

    if(flood[posX][posY] == 0 || isGoal(posX, posY)){
      goalReached = true;
      Serial.println("=== GOAL REACHED! ===");
      return;
    }

    senseWalls(posX, posY, heading);
    floodFill();
    printFlood();

    int dir = selectDirection(posX, posY, heading);
    Serial.print("Best dir="); Serial.println(dir);

    if(dir == -1) dir = (heading + 2) % 4;

    if(dir != heading){
      turnToFace(dir);
      resumeDriving();
    } else {
      Serial.println(">> CONTINUE STRAIGHT");
      yaw = 0;
      lastUs = micros();
      delay(100);
      setForward();
      analogWrite(ENA, baseA);
      analogWrite(ENB, baseB);
    }
  }
}
