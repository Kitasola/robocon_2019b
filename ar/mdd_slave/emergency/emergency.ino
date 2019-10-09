constexpr int NUM_ROBOT_SW = 2, NUM_EMERGENCY = 1, NUM_CHECK_LED = 1;
constexpr int REMOTE = 6, ROBOT[NUM_ROBOT_SW] = {7, 8}, EMERGENCY[NUM_EMERGENCY] = {12}, CHECK_LED[NUM_CHECK_LED] = {11}, ACT = 13;
void setup() {
  pinMode(REMOTE, INPUT_PULLUP);
  for (int i = 0; i < NUM_ROBOT_SW; ++i) {
    pinMode(ROBOT[i], INPUT_PULLUP);
  }
  for(int i = 0; i < NUM_EMERGENCY; ++i) {
    pinMode(EMERGENCY[i], OUTPUT);
  }
  for(int i = 0; i < NUM_CHECK_LED; ++i) {
    pinMode(CHECK_LED[i], OUTPUT);
  }
  pinMode(ACT, OUTPUT);
  digitalWrite(ACT, 1);
}

bool run = false, current_robot_level = false, prev_robot_level = false;
void loop() {
  // 押されていればfalse
  bool remote_level = digitalRead(REMOTE);
  prev_robot_level = current_robot_level;
  current_robot_level = true;
  for(int i = 0; i < NUM_ROBOT_SW; ++i) {
    current_robot_level = current_robot_level && digitalRead(ROBOT[i]);
  }
  
  if(run) {
    if(remote_level && current_robot_level) {
      run = true;
    } else {
      run = false;    
    }
  } else {
    if(remote_level && current_robot_level && !prev_robot_level) {
      run = true;        
    }else {
      run = false;
    }
  }

  for(int i  = 0; i < NUM_EMERGENCY; ++i){
    digitalWrite(EMERGENCY[i], run);
  }
  digitalWrite(ACT, run);
  digitalWrite(CHECK_LED[0], current_robot_level);
}
