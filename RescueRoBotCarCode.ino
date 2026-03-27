#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h> 

// ========== WIFI CONFIGURATION ==========
const char* apSsid     = "Robot_Tham_Do";
const char* apPassword = "thebeacroxtheic161093";
WebServer server(80);

// ========== PIN DECLARATIONS ==========
#define ENA_PIN 14
#define IN1     27
#define IN2     26
#define IN3     25
#define IN4     33
#define ENB_PIN 32
#define TRIG_PIN 4
#define ECHO_PIN 5
#define MQ2_PIN  36  
#define SERVO_PIN 13 

// ========== PWM SETTINGS (ESP32 Core 3.x) ==========
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;    

uint8_t manualSpeed = 255; 
uint8_t autoSpeed   = 200;  
int GAS_FLEE        = 1800; 
int GAS_TRACK       = 800;  

int currentMode = 0; 
// 0: Manual 
// 1: BÁM TƯỜNG (Wall Follower SLAM)
// 2: TÌM GAS (Bám tường + Sniffing)
// 3: BỎ CHẠY (Bám tường + Flee)
// 4: NÉ VẬT CẢN (Quét Radar Trái/Phải/Quay đầu)

unsigned long lastPingTime = 0;
bool hasConnected = false; // CỜ TRẠNG THÁI: Xác định xe đã từng kết nối web chưa
char currentManualCmd = 's'; 

// Non-blocking Timers
unsigned long wTimer = 0;
unsigned long oTimer = 0;

// Trạng thái Bám Tường
enum WallState { LOOK_FRONT, LOOK_LEFT, DO_MOVE, TURN_RIGHT, TURN_LEFT, FLEEING, SNIFF_L, SNIFF_R, SNIFF_FWD };
WallState wState = LOOK_FRONT;

// Trạng thái Né Vật Cản (Cải tiến với bước lùi lại - O_BACKWARD)
enum ObsState { O_FWD, O_BACKWARD, O_SCAN_R, O_SCAN_L, O_DECIDE, O_TURN };
ObsState oState = O_FWD;
int oTurnDir = 0; // 3: Trái, 4: Phải, 5: Quay đầu

// Sensor Data
float currentDist = 0;
int currentGas = 0;
unsigned long lastSensorRead = 0;

// Servo & Radar
Servo radarServo;
float distLeft = 0;
float distRight = 0;
float distFront = 0;
int currentServoAngle = 135; // TÂM CHO SERVO 270 ĐỘ (270 / 2)

int sniffLeftGas = 0;
int sniffRightGas = 0;

// ========== WEB INTERFACE (HTML/CSS/JS) ==========
const char index_html[] PROGMEM = R"=====(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rescue Robot UI</title>
<style>
  body { font-family: sans-serif; background: #0a0a0a; color: #fff; text-align: center; margin: 0; padding: 5px; }
  .panel { background: #1a1a1a; border: 1px solid #333; border-radius: 8px; padding: 10px; margin: 5px auto; max-width: 400px; }
  .bar-bg { background: #333; border-radius: 5px; width: 100%; height: 18px; position: relative; overflow: hidden; margin-top: 3px; }
  .bar-fill { height: 100%; width: 0%; transition: width 0.3s; }
  .btn { width: 70px; height: 50px; margin: 2px; font-weight: bold; border-radius: 8px; background: #333; color: white; border: none;}
  .btn:active { background: #00ffcc; color: #000; }
  .btn-stop { background: #cc0000; }
  .mode-btn { width: 48%; padding: 8px; font-size: 11px; font-weight: bold; border-radius: 5px; border:none; cursor: pointer; margin: 2px; }
  .mode-active { background: #00aaff; color: #fff; box-shadow: 0 0 10px #00aaff;}
  .mode-off { background: #444; color: #aaa; }
  .btn-refresh { background: #ff5555; color: white; width: 98%; margin-top: 5px; }
  .btn-refresh:active { background: #cc0000; }
  .slider { width: 100%; margin-top: 5px; }
  #map-wrapper { background: #000; border: 2px solid #555; width: 100%; height: 250px; position: relative; overflow: hidden; margin-top: 5px; border-radius: 5px;}
  canvas { background: transparent; display: block; width: 100%; height: 100%;}
  .legend { font-size: 10px; color: #aaa; margin-top: 3px; }
</style>
</head><body>
  <h2 style="color: #00ffcc; margin: 5px 0; font-size: 18px;">ROBOT CONTROL CENTER</h2>
  <div class="panel">
    <div style="text-align: left; font-size: 12px;">
      <b>Gas Level:</b> <span id="gas-val">0</span>
      <div class="bar-bg"><div id="gas-bar" class="bar-fill" style="background:#00cc66;"></div></div>
      <b style="margin-top:3px; display:inline-block;">Radar Dist:</b> <span id="dist-val">0</span> cm
      <div class="bar-bg"><div id="dist-bar" class="bar-fill" style="background:#00aaff;"></div></div>
    </div>
  </div>

  <div class="panel">
    <button id="btn-m0" class="mode-btn mode-active" onclick="setMode(0)">0: MANUAL</button>
    <button id="btn-m4" class="mode-btn mode-off" onclick="setMode(4)">4: NÉ VẬT CẢN (QUÉT)</button>
    <button id="btn-m1" class="mode-btn mode-off" onclick="setMode(1)">1: BÁM TƯỜNG (SLAM)</button>
    <button id="btn-m2" class="mode-btn mode-off" onclick="setMode(2)">2: BÁM TƯỜNG + TÌM GAS</button>
  </div>

  <div class="panel" id="manual-controls">
    <div><button class="btn" onmousedown="cmd('f')" onmouseup="cmd('s')" ontouchstart="cmd('f')" ontouchend="cmd('s')">FWD</button></div>
    <div>
      <button class="btn" onmousedown="cmd('l')" onmouseup="cmd('s')" ontouchstart="cmd('l')" ontouchend="cmd('s')">LEFT</button>
      <button class="btn btn-stop" onmousedown="cmd('s')" ontouchstart="cmd('s')">STOP</button>
      <button class="btn" onmousedown="cmd('r')" onmouseup="cmd('s')" ontouchstart="cmd('r')" ontouchend="cmd('s')">RIGHT</button>
    </div>
    <div><button class="btn" onmousedown="cmd('b')" onmouseup="cmd('s')" ontouchstart="cmd('b')" ontouchend="cmd('s')">BWD</button></div>
    
    <div style="font-size: 12px; margin-top:8px; text-align: left;">
      <b>Radar Angle (Chỉ dùng Mode 0):</b> <span id="ang-val">135</span>&deg;
      <input type="range" min="0" max="270" value="135" class="slider" id="servoSlider" onchange="fetch('/servo?ang='+this.value); document.getElementById('ang-val').innerText=this.value;">
    </div>
  </div>

  <div class="panel" style="padding: 5px;">
    <h3 style="margin:2px 0; font-size: 12px; color:#fff;">Live SLAM Map</h3>
    <button class="mode-btn btn-refresh" onclick="refreshMap()">🔄 REFRESH DATA & MAP</button>
    <div id="map-wrapper"><canvas id="slamMap" width="400" height="400"></canvas></div>
    <div class="legend">🟦 Đường đi | ⬜ Tường | 🔴 Nguy hiểm (>800)</div>
  </div>

<script>
  let currentCmd = 's';
  let servoAng = 135;
  const canvas = document.getElementById('slamMap');
  const ctx = canvas.getContext('2d');
  let robX = 200, robY = 200, heading = 0; 
  let path = [{x: 200, y: 200}];
  let walls = [];
  let maxGas = 0; let maxGasPos = {x: 200, y: 200}; let gasZones = []; 

  function cmd(dir) { currentCmd = dir; fetch('/cmd?dir='+dir); }
  function setMode(m) {
    fetch('/mode?m='+m);
    for(let i=0; i<=4; i++) {
      let btn = document.getElementById('btn-m'+i);
      if(btn) btn.className = (m==i) ? 'mode-btn mode-active' : 'mode-btn mode-off';
    }
    document.getElementById('servoSlider').disabled = (m != 0);
  }

  // Chức năng Refresh toàn bộ dữ liệu
  function refreshMap() {
    robX = 200; robY = 200; heading = 0;
    path = [{x: 200, y: 200}];
    walls = [];
    gasZones = [];
    maxGas = 0;
    cmd('s'); // Dừng xe
    drawMap();
  }

  function drawMap() {
    ctx.clearRect(0, 0, 400, 400); 
    gasZones.forEach(z => {
      ctx.beginPath(); ctx.arc(z.x, z.y, z.r, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(255, 0, 0, 0.3)'; ctx.fill();
    });
    ctx.beginPath(); ctx.moveTo(path[0].x, path[0].y);
    for(let i=1; i<path.length; i++) ctx.lineTo(path[i].x, path[i].y);
    ctx.strokeStyle = '#00aaff'; ctx.lineWidth = 2; ctx.stroke();
    
    ctx.fillStyle = '#ffffff';
    walls.forEach(w => { ctx.fillRect(w.x - 1, w.y - 1, 3, 3); });
    
    ctx.save(); ctx.translate(robX, robY); ctx.rotate(heading * Math.PI / 180);
    ctx.beginPath(); ctx.moveTo(0, -6); ctx.lineTo(-4, 4); ctx.lineTo(4, 4);
    ctx.fillStyle = '#ffcc00'; ctx.fill(); ctx.restore();
  }

  setInterval(() => {
    fetch('/ping').then(r => r.json()).then(d => {
      document.getElementById('dist-val').innerText = d.dist.toFixed(1);
      document.getElementById('dist-bar').style.width = Math.min((d.dist/200)*100, 100) + '%';
      document.getElementById('gas-val').innerText = d.gas;
      document.getElementById('gas-bar').style.width = Math.min((d.gas/4095)*100, 100) + '%';
      
      let gBar = document.getElementById('gas-bar');
      if(d.gas > 1800) gBar.style.background = '#ff0000';
      else if(d.gas > 800) gBar.style.background = '#ff9900';
      else gBar.style.background = '#00cc66';

      if(d.mode != 0) { currentCmd = d.action; servoAng = d.servo; } 
      
      let moved = false; let speedFactor = 4; 
      if(currentCmd === 'f') { robX += Math.sin(heading * Math.PI/180)*speedFactor; robY -= Math.cos(heading * Math.PI/180)*speedFactor; moved=true;}
      else if(currentCmd === 'b') { robX -= Math.sin(heading * Math.PI/180)*speedFactor; robY += Math.cos(heading * Math.PI/180)*speedFactor; moved=true;}
      else if(currentCmd === 'l') { heading -= 15; moved=true;}
      else if(currentCmd === 'r') { heading += 15; moved=true;}
      if(moved) path.push({x: robX, y: robY});

      if(d.dist < 60 && d.dist > 2) {
        // Cập nhật lại logic góc chiếu bản đồ cho tâm 135
        let absAngle = heading + (servoAng - 135); 
        let rad = absAngle * Math.PI / 180;
        let wallX = robX + Math.sin(rad) * (d.dist/2); 
        let wallY = robY - Math.cos(rad) * (d.dist/2);
        walls.push({x: wallX, y: wallY});
      }

      if (d.gas > 800) {
        if (d.gas > maxGas) { maxGas = d.gas; maxGasPos = {x: robX, y: robY}; }
      } else if (maxGas > 1000 && d.gas < 700) {
        gasZones.push({x: maxGasPos.x, y: maxGasPos.y, r: Math.min(maxGas / 30, 60)}); maxGas = 0; 
      }
      drawMap();
    }).catch(e => console.log('Ping failed')); 
  }, 500); 
</script></body></html>
)=====";

// ========== MOTOR CORE ==========
void setMotor(uint8_t speedA, uint8_t speedB) {
  ledcWrite(ENA_PIN, speedA);
  ledcWrite(ENB_PIN, speedB);
}

void drive(int dir, uint8_t speed) {
  switch(dir) {
    case 0: digitalWrite(IN1, 0); digitalWrite(IN2, 0); digitalWrite(IN3, 0); digitalWrite(IN4, 0); setMotor(0,0); break;
    case 1: digitalWrite(IN1, 1); digitalWrite(IN2, 0); digitalWrite(IN3, 0); digitalWrite(IN4, 1); setMotor(speed, speed); break;
    case 2: digitalWrite(IN1, 0); digitalWrite(IN2, 1); digitalWrite(IN3, 1); digitalWrite(IN4, 0); setMotor(speed, speed); break;
    case 3: digitalWrite(IN1, 1); digitalWrite(IN2, 0); digitalWrite(IN3, 1); digitalWrite(IN4, 0); setMotor(speed, speed); break;
    case 4: digitalWrite(IN1, 0); digitalWrite(IN2, 1); digitalWrite(IN3, 0); digitalWrite(IN4, 1); setMotor(speed, speed); break;
  }
}

void setRadar(int angle) {
  if (currentServoAngle != angle) {
    radarServo.write(angle);
    currentServoAngle = angle;
  }
}

// ========== SENSOR READING ==========
float getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); 
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

void readSensors() {
  if (millis() - lastSensorRead > 80) {
    currentGas = analogRead(MQ2_PIN);
    currentDist = getDistanceCm();
    lastSensorRead = millis();
  }
}

// ========== HTTP ROUTERS ==========
void handleRoot() { server.send_P(200, "text/html", index_html); }
void handlePing() {
  lastPingTime = millis(); 
  hasConnected = true; // Đánh dấu là đã có kết nối web
  
  String action = "s";
  if(currentMode >= 1 && currentMode <= 3) {
    if(wState == DO_MOVE || wState == SNIFF_FWD) action = "f"; 
    else if(wState == FLEEING) action = "b";
    else if(wState == TURN_LEFT || wState == SNIFF_L) action = "l";
    else if(wState == TURN_RIGHT || wState == SNIFF_R) action = "r";
  } else if (currentMode == 4) {
    if(oState == O_FWD) action = "f";
    else if(oState == O_BACKWARD) action = "b";
    else if(oState == O_TURN && oTurnDir == 3) action = "l";
    else if(oState == O_TURN && oTurnDir == 4) action = "r";
    else if(oState == O_TURN && oTurnDir == 5) action = "b";
  } else {
    action = String(currentManualCmd);
  }
  
  String json = "{\"gas\":" + String(currentGas) + ",\"dist\":" + String(currentDist) + ",\"mode\":" + String(currentMode) + ",\"action\":\"" + action + "\",\"servo\":" + String(currentServoAngle) + "}";
  server.send(200, "application/json", json);
}
void handleCmd() { if (server.hasArg("dir")) currentManualCmd = server.arg("dir")[0]; server.send(200, "text/plain", "OK"); }
void handleServo() { 
  if (server.hasArg("ang") && currentMode == 0) {
    setRadar(server.arg("ang").toInt()); 
  }
  server.send(200, "text/plain", "OK"); 
}
void handleMode() {
  if (server.hasArg("m")) {
    currentMode = server.arg("m").toInt();
    wState = LOOK_FRONT; oState = O_FWD; drive(0, 0); setRadar(135);
  }
  server.send(200, "text/plain", "OK");
}

// ========== MODE 4: NÉ VẬT CẢN (CẢI TIẾN THEO LOGIC MỚI LÙI TRƯỚC KHI QUÉT) ==========
void handleObstacleAvoidance() {
  unsigned long now = millis();

  switch (oState) {
    case O_FWD:
      setRadar(135); // Tâm servo 270 là 135
      drive(1, autoSpeed); // Đi thẳng
      if (now - oTimer > 150) { 
        if (currentDist > 0 && currentDist < 30.0) {
          drive(2, autoSpeed); // Lập tức lùi nhẹ thay vì chỉ phanh
          oState = O_BACKWARD;
          oTimer = now;
        } else {
          oTimer = now; 
        }
      }
      break;

    case O_BACKWARD:
      // Lùi lại 250ms giống code của bạn để tăng góc nhìn
      if (now - oTimer > 250) {
        drive(0,0); // Dừng lại
        oState = O_SCAN_R;
        oTimer = now;
      }
      break;

    case O_SCAN_R:
      setRadar(45); // 135 - 90 = 45 (Nhìn phải vuông góc)
      if (now - oTimer > 400) { 
        distRight = getDistanceCm();
        oState = O_SCAN_L;
        oTimer = now;
      }
      break;

    case O_SCAN_L:
      setRadar(225); // 135 + 90 = 225 (Nhìn trái vuông góc)
      if (now - oTimer > 500) { 
        distLeft = getDistanceCm();
        oState = O_DECIDE;
        oTimer = now;
      }
      break;

    case O_DECIDE:
      setRadar(135); // Trả thẳng cổ
      if (distLeft < 25.0 && distRight < 25.0) {
        oTurnDir = 5; // Cả 2 bên kẹt -> Quay đầu
        drive(4, autoSpeed + 20); 
      } else if (distRight >= distLeft) {
        oTurnDir = 4; // Phải rộng hơn -> Rẽ phải
        drive(4, autoSpeed + 20);
      } else {
        oTurnDir = 3; // Trái rộng hơn -> Rẽ trái
        drive(3, autoSpeed + 20);
      }
      oState = O_TURN;
      oTimer = now;
      break;

    case O_TURN:
      int turnWait = (oTurnDir == 5) ? 900 : 450; 
      if (now - oTimer > turnWait) {
        drive(0,0);
        oState = O_FWD;
        oTimer = now;
      }
      break;
  }
}

// ========== MODE 1, 2, 3: BÁM TƯỜNG (SLAM) ==========
void handleWallFollower(bool canSniff, bool canFlee) {
  unsigned long now = millis();

  if (canFlee && currentGas > GAS_FLEE && wState != FLEEING) { wState = FLEEING; wTimer = now; }
  if (canSniff && currentGas > GAS_TRACK && wState < SNIFF_L) { wState = SNIFF_L; wTimer = now; setRadar(135); drive(0,0); }

  switch (wState) {
    case LOOK_FRONT:
      setRadar(135); // Cổ thẳng
      if (now - wTimer > 300) { 
        distFront = getDistanceCm();
        if (distFront > 0 && distFront < 20.0) { 
           wState = TURN_RIGHT; wTimer = now;
        } else {
           wState = LOOK_LEFT; wTimer = now;
        }
      }
      break;

    case LOOK_LEFT:
      setRadar(225); // Vuông góc bên trái
      if (now - wTimer > 300) { 
        distLeft = getDistanceCm();
        wState = DO_MOVE; wTimer = now;
      }
      break;

    case DO_MOVE:
      if (distLeft > 0 && distLeft < 10.0) drive(4, autoSpeed + 20); // Ép phải
      else if (distLeft > 25.0 && distLeft < 50.0) drive(3, autoSpeed + 20); // Ôm trái
      else drive(1, autoSpeed + 30); 

      if (now - wTimer > 800) { 
        drive(0,0); wState = LOOK_FRONT; wTimer = now;
      }
      break;

    case TURN_RIGHT:
      drive(4, autoSpeed + 30); 
      if (now - wTimer > 600) { drive(0,0); wState = LOOK_FRONT; wTimer = now; }
      break;

    case FLEEING:
      drive(3, 255); 
      if (now - wTimer > 800) { drive(0,0); wState = LOOK_FRONT; }
      break;

    case SNIFF_L:
      drive(3, autoSpeed + 20); 
      if (now - wTimer > 250) { drive(0,0); sniffLeftGas = currentGas; wState = SNIFF_R; wTimer = now; }
      break;
    case SNIFF_R:
      drive(4, autoSpeed + 20); 
      if (now - wTimer > 500) { 
        drive(0,0); sniffRightGas = currentGas; 
        if (sniffLeftGas > sniffRightGas) { drive(3, autoSpeed+20); delay(250); } 
        wState = SNIFF_FWD; wTimer = now;
      }
      break;
    case SNIFF_FWD:
      drive(1, autoSpeed);
      if (now - wTimer > 600) { drive(0,0); wState = LOOK_FRONT; } 
      break;
  }
}

// ========== SETUP & MAIN LOOP ==========
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES);

  // Khởi tạo PWM về 0 ngay từ đầu để chống trôi motor
  setMotor(0,0);

  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  radarServo.setPeriodHertz(50); 
  radarServo.attach(SERVO_PIN, 500, 2400); 
  radarServo.write(135); // Tâm 135

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); pinMode(MQ2_PIN, INPUT);

  WiFi.mode(WIFI_AP); WiFi.softAP(apSsid, apPassword);
  server.on("/", handleRoot); server.on("/ping", handlePing);
  server.on("/cmd", handleCmd); server.on("/mode", handleMode); server.on("/servo", handleServo);
  server.begin();
  lastPingTime = millis();
}

void loop() {
  server.handleClient();
  readSensors();
  unsigned long now = millis();

  // === LOGIC BẢO VỆ MỚI (FAILSAFE) ===
  if (now - lastPingTime > 2500) { 
    if (hasConnected) {
      // Đã từng kết nối nhưng mất sóng -> Lùi thoát hiểm
      drive(2, autoSpeed); 
    } else {
      // Chưa từng kết nối (mới bật nguồn) -> Đứng im an toàn
      drive(0, 0); 
    }
    return; // Dừng việc thực thi các mode bên dưới
  } 

  if (currentMode == 1) handleWallFollower(false, false); 
  else if (currentMode == 2) handleWallFollower(true, false);  
  else if (currentMode == 3) handleWallFollower(false, true);  
  else if (currentMode == 4) handleObstacleAvoidance(); 
  else {
    // Mode 0: Manual
    if (currentManualCmd == 'f') drive(1, manualSpeed);
    else if (currentManualCmd == 'b') drive(2, manualSpeed);
    else if (currentManualCmd == 'l') drive(3, manualSpeed);
    else if (currentManualCmd == 'r') drive(4, manualSpeed);
    else drive(0, 0);
  }
}


