#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>


// Define Pins
#define infeedStartPin 35
#define infeedStopPin 34
#define infeedReversePin 32
#define infeedIncreaseSpeedPin 33
#define infeedDecreaseSpeedPin 25
#define engineFanPin 27
#define infeedConveyorPin 26

// Define PWM Channels
#define PWM_FREQUENCY 1000 // 1kHz
#define PWM_RESOLUTION 8    // 8-bit resolution
#define PWM_CHANNEL_FAN 0
#define PWM_CHANNEL_INFEED 1
#define DUTY_CYCLE_2_5V 85  // Corresponding to 2.5V

// Define debounce time interval in milliseconds
#define DEBOUNCE_INTERVAL 500

// Variables to hold last debounce times for each button
unsigned long lastDebounceTimeStart = 0;
unsigned long lastDebounceTimeStop = 0;
unsigned long lastDebounceTimeReverse = 0;
unsigned long lastDebounceTimeIncrease = 0;
unsigned long lastDebounceTimeDecrease = 0;

unsigned long previousMillisInfeed = 0;
const long intervalInfeed = 20; // 20 ms
const float voltageAdjustment = 0.1; // Global constant for voltage adjustment
const long intervalEngineFan = 500; // 500 ms
unsigned long previousMillisEngineFan = 0;
const long forwardTimeout = 60000; // 1 minute in milliseconds
unsigned long lastForwardTime = 0; // To keep track of the last time the fan was in forward mode.

const char* ssid = "Falcon Wifi";
const char* password = "falcon73";
const char* hostname = "peterson4700";
AsyncWebServer server(80); // Create an instance of the server on port 80


enum MachineState {
  REVERSE,
  NEUTRAL,
  FORWARD
};

const float neutralVoltage = 2.5; 
const float maxDutyCycle = 180;

float defaultInfeedReverseVoltage = 0.5; // in volts
float defaultInfeedForwardVoltage = 2.9; // in volts
float currentInfeedVoltage = neutralVoltage; // initialized to 2.5V, neutral state
float targetInfeedVoltage = neutralVoltage; // Initially neutral
int infeedSpeed = 0;

float defaultFanReverseVoltage = 0; // in volts
float defaultFanForwardVoltage = 5; // in volts
float currentEngineFanVoltage = neutralVoltage; // initialized to 2.5V, neutral state
float targetEngineFanVoltage = neutralVoltage; // Initially neutral
int fanSpeed = 0;


// Define current states of the machine
MachineState engineFanState = NEUTRAL;
MachineState infeedConveyorState = NEUTRAL;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setHostname(hostname);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (!MDNS.begin(hostname)) { // Start the mDNS responder for peterson4700.local
  Serial.println("Error setting up MDNS responder!");
} else {
  Serial.println("mDNS responder started");
}

  // Set pin modes
  pinMode(infeedStartPin, INPUT);
  pinMode(infeedStopPin, INPUT);
  pinMode(infeedReversePin, INPUT);
  pinMode(infeedIncreaseSpeedPin, INPUT);
  pinMode(infeedDecreaseSpeedPin, INPUT);

  // Setup PWM for Engine Fan
  ledcSetup(PWM_CHANNEL_FAN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(engineFanPin, PWM_CHANNEL_FAN);
  ledcWrite(PWM_CHANNEL_FAN, DUTY_CYCLE_2_5V); // Initialize to 2.5V
  
  // Setup PWM for Infeed Conveyor
  ledcSetup(PWM_CHANNEL_INFEED, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(infeedConveyorPin, PWM_CHANNEL_INFEED);
  ledcWrite(PWM_CHANNEL_INFEED, DUTY_CYCLE_2_5V); // Initialize to 2.5V

  ledcWrite(PWM_CHANNEL_FAN, calculateDutyCycle(currentEngineFanVoltage));
  ledcWrite(PWM_CHANNEL_INFEED, calculateDutyCycle(currentInfeedVoltage));

  // Define the routes and start the server
  serverHandlers();
  server.begin();
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  lastForwardTime = millis();
}

void loop() {
  handleButtonPress();
  updateInfeedVoltage();
  updateEngineFanVoltage();
  checkFanForwardTimeout();
  ArduinoOTA.handle();
  // Your other logic here...
}

void checkFanForwardTimeout() {
    if (engineFanState != FORWARD) {
        if (millis() - lastForwardTime >= forwardTimeout) {
            Serial.println("Fan has been in non-forward mode for 1 minute. Setting to forward mode.");
            fanForward();
        }
    }
}

void handleButtonPress() {
  unsigned long currentMillis = millis();

  // Handle Infeed Start
  static bool prevStartState = LOW;
  bool startState = digitalRead(infeedStartPin);
  if(startState == HIGH && prevStartState == LOW && currentMillis - lastDebounceTimeStart > DEBOUNCE_INTERVAL) {
    Serial.println("Infeed Start Pressed");
    infeedStart();
    lastDebounceTimeStart = currentMillis;
  }
  prevStartState = startState;

  // Handle Infeed Stop
  static bool prevStopState = LOW;
  bool stopState = digitalRead(infeedStopPin);
  if(stopState == HIGH && prevStopState == LOW && currentMillis - lastDebounceTimeStop > DEBOUNCE_INTERVAL) {
    Serial.println("Infeed Stop Pressed");
    infeedStop();
    lastDebounceTimeStop = currentMillis;
  }
  prevStopState = stopState;

  // ... Similar for other buttons
  // Handle Infeed Reverse
  static bool prevReverseState = LOW;
  bool reverseState = digitalRead(infeedReversePin);
  if(reverseState == HIGH && prevReverseState == LOW && currentMillis - lastDebounceTimeReverse > DEBOUNCE_INTERVAL) {
    Serial.println("Infeed Reverse Pressed");
    infeedReverse();
    lastDebounceTimeReverse = currentMillis;
  }
  prevReverseState = reverseState;

  // Handle Infeed Increase Speed
  static bool prevIncreaseState = LOW;
  bool increaseState = digitalRead(infeedIncreaseSpeedPin);
  if(increaseState == HIGH && prevIncreaseState == LOW && currentMillis - lastDebounceTimeIncrease > DEBOUNCE_INTERVAL) {
    Serial.println("Infeed Increase Speed Pressed");
    infeedIncrease();
    lastDebounceTimeIncrease = currentMillis;
  }
  prevIncreaseState = increaseState;

  // Handle Infeed Decrease Speed
  static bool prevDecreaseState = LOW;
  bool decreaseState = digitalRead(infeedDecreaseSpeedPin);
  if(decreaseState == HIGH && prevDecreaseState == LOW && currentMillis - lastDebounceTimeDecrease > DEBOUNCE_INTERVAL) {
    Serial.println("Infeed Decrease Speed Pressed");
    infeedDecrease();
    lastDebounceTimeDecrease = currentMillis;
  }
  prevDecreaseState = decreaseState;
}

int calculateDutyCycle(float voltage) {
  if (voltage <= neutralVoltage) 
    return (int)(voltage * (DUTY_CYCLE_2_5V / neutralVoltage));
  else
    return DUTY_CYCLE_2_5V + (int)((voltage - neutralVoltage) * ((maxDutyCycle - DUTY_CYCLE_2_5V) / (neutralVoltage)));
}

// Function to gradually change the voltage towards the target voltage
void updateEngineFanVoltage() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisEngineFan >= intervalEngineFan) {
    previousMillisEngineFan = currentMillis;
    if (abs(currentEngineFanVoltage - targetEngineFanVoltage) > 0.01) { // To avoid float comparison issues
      float increment = (currentEngineFanVoltage < targetEngineFanVoltage) ? 0.1 : -0.1;
      currentEngineFanVoltage += increment;
      ledcWrite(PWM_CHANNEL_FAN, calculateDutyCycle(currentEngineFanVoltage));
      Serial.print("Current Engine Fan Voltage: ");
      Serial.print(currentEngineFanVoltage);
      Serial.print(" Duty Cycle: ");
      Serial.println(calculateDutyCycle(currentEngineFanVoltage));
      fanSpeed = (abs(currentEngineFanVoltage - neutralVoltage)*10)+1;
    }
    updateEngineFanState();
  }
}

// Function to gradually change the voltage towards the target voltage
void updateInfeedVoltage() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisInfeed >= intervalInfeed) {
    previousMillisInfeed = currentMillis;
    if (abs(currentInfeedVoltage - targetInfeedVoltage) > 0.01) { // To avoid float comparison issues
      float increment = (currentInfeedVoltage < targetInfeedVoltage) ? 0.1 : -0.1;
      currentInfeedVoltage += increment;
      ledcWrite(PWM_CHANNEL_INFEED, calculateDutyCycle(currentInfeedVoltage));
      Serial.print("Current Infeed Voltage: ");
      Serial.print(currentInfeedVoltage);
      Serial.print(" Duty Cycle: ");
      Serial.println(calculateDutyCycle(currentInfeedVoltage));
      infeedSpeed = (abs(currentInfeedVoltage - neutralVoltage)*10)+1;
    }
    updateInfeedConveyorState();
  }
}

// Function to update infeed conveyor state
void updateInfeedConveyorState() {
    static MachineState previousState = NEUTRAL; // Keeps track of the previous state between function calls
    
    MachineState newState = infeedConveyorState; // Assign current state to new state as a starting point
    
    if(currentInfeedVoltage < neutralVoltage) {
        newState = REVERSE;
    } else if(currentInfeedVoltage > neutralVoltage) {
        newState = FORWARD;
    } else {
        newState = NEUTRAL;
    }

    if(newState != previousState) { // If the state has changed, update the state and print to Serial
        infeedConveyorState = newState; // Update the actual state of the Conveyor
        Serial.print("Infeed Conveyor State Changed: ");
        switch(infeedConveyorState) {
            case FORWARD: Serial.println("FORWARD"); break;
            case REVERSE: Serial.println("REVERSE"); break;
            case NEUTRAL: Serial.println("NEUTRAL"); break;
        }
        previousState = newState; // Update the previous state for the next comparison
    }
}

void updateEngineFanState() {
    static MachineState previousState = FORWARD; // Initialize with FORWARD as it is the starting state.
    MachineState newState = engineFanState;
    
    if(currentEngineFanVoltage < neutralVoltage) {
        newState = REVERSE;
    } else if(currentEngineFanVoltage > neutralVoltage) {
        newState = FORWARD;
    } else {
        newState = NEUTRAL;
    }

    if(newState != previousState) { 
        engineFanState = newState;
        Serial.print("Engine Fan State Changed: ");
        switch(engineFanState) {
            case FORWARD: 
                Serial.println("FORWARD"); 
                break;
            case REVERSE: 
                Serial.println("REVERSE");
                lastForwardTime = millis(); // Update the last forward time when leaving FORWARD state.
                break;
            case NEUTRAL: 
                Serial.println("NEUTRAL");
                lastForwardTime = millis(); // Update the last forward time when leaving FORWARD state.
                break;
        }
        previousState = newState; // Update the previous state for the next comparison.
    }
}



void infeedReverse() {
  targetInfeedVoltage = defaultInfeedReverseVoltage;
}

void infeedStart() {
  targetInfeedVoltage = defaultInfeedForwardVoltage;
}

void infeedStop() {
  targetInfeedVoltage = neutralVoltage; // Neutral voltage
}

void infeedIncrease() {
    if (infeedConveyorState == REVERSE) {
        targetInfeedVoltage -= voltageAdjustment;
        if(targetInfeedVoltage < 0.0) targetInfeedVoltage = 0.0; // Ensure it doesn't go below 0V
    }
    else if (infeedConveyorState == FORWARD) {
        targetInfeedVoltage += voltageAdjustment;
        if(targetInfeedVoltage > 5.0) targetInfeedVoltage = 5.0; // Ensure it doesn't exceed 5V
    }
}

void infeedDecrease() {
    if (infeedConveyorState == REVERSE) {
        targetInfeedVoltage += voltageAdjustment;
        if(targetInfeedVoltage > 5.0) targetInfeedVoltage = 5.0; // Ensure it doesn't exceed 5V
    }
    else if (infeedConveyorState == FORWARD) {
        targetInfeedVoltage -= voltageAdjustment;
        if(targetInfeedVoltage < 0.0) targetInfeedVoltage = 0.0; // Ensure it doesn't go below 0V
    }
}

void fanForward() {
    targetEngineFanVoltage = defaultFanForwardVoltage;
}

void fanReverse() {
    targetEngineFanVoltage = defaultFanReverseVoltage;
    lastForwardTime = millis(); // Update the last forward time when the fanReverse function is called.
}

void fanStop() {
    targetEngineFanVoltage = neutralVoltage;
    lastForwardTime = millis(); // Update the last forward time when the fanStop function is called.
}

void handleInfeedReverseButton(AsyncWebServerRequest *request) {
    infeedReverse();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleInfeedStartButton(AsyncWebServerRequest *request) {
    infeedStart();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleInfeedStopButton(AsyncWebServerRequest *request) {
    infeedStop();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleInfeedIncreaseButton(AsyncWebServerRequest *request) {
    infeedIncrease();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleInfeedDecreaseButton(AsyncWebServerRequest *request) {
    infeedDecrease();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleFanForwardButton(AsyncWebServerRequest *request) {
    fanForward();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleFanReverseButton(AsyncWebServerRequest *request) {
    fanReverse();
    request->send(200, "text/plain", "OK");  // Send Response
}

void handleFanStopButton(AsyncWebServerRequest *request) {
    fanStop();
    request->send(200, "text/plain", "OK");  // Send Response
}


void handleStatus(AsyncWebServerRequest *request) {
  String statusJson = "{";
  statusJson += "\"infeedConveyorState\":\"" + String(getInfeedConveyorState()) + "\",";
  statusJson += "\"engineFanState\":\"" + String(getEngineFanState()) + "\"";
  statusJson += "}";
  request->send(200, "application/json", statusJson); // Send the response to the client as JSON
}

void handleRoot(AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Machine Status</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"; // Add this line for mobile responsiveness

    // CSS styles
    html += "<style>";
    html += "body { font-family: Arial, Helvetica, sans-serif; margin: 20px; padding: 20px; font-size: 20px; }"; // Larger font size
    html += "h1 { color: #4CAF50; font-size: 36px; }";  // Larger header
    html += "h2 { color: #2196F3; font-size: 28px; }";  // Larger sub-header
    html += "p { font-size: 12px; }"; 
    html += "button { padding: 20px 24px; margin: 16px 0; border: 1px; cursor: pointer; width: 100%; font-size: 24px; border-radius: 12px; font-weight: bold; }"; // Larger button and text
    html += "button.forward { background-color: #4CAF50; color: white; }";
    html += "button.stop { background-color: #f44336; color: white; }";
    html += "button.reverse { background-color: #ffeb3b; color: black; }";
    html += "button:hover { opacity: 0.8; }";
    html += ".section-box { border: 2px solid #ccc; padding: 20px; margin: 20px 0; border-radius: 10px; }"; // Add this line for the section box
    html += "</style>";
    html += "</head><body>";

    // Header
    html += "<h1>Peterson 4700</h1>";
    html += "<p>Device IP Address: " + WiFi.localIP().toString() + "</p>";
    
    // Infeed Status and Control Buttons
    html += "<div class='section-box'>";
    html += "<h2>Infeed Section</h2>";
    html += "<h3 id='infeedStatus'>Infeed Status: " + String(getInfeedConveyorState()) + "</h3>";
    html += "<p> Infeed default forward speed = 4. Max = 25</p>";
    html += "<p> Infeed default reverse speed = 20. Max = 25</p>";
    html += "<button class='forward' onclick='sendRequest(\"/infeed_start\")'>Forward</button>";
    html += "<button class='stop' onclick='sendRequest(\"/infeed_stop\")'>Stop</button>";
    html += "<button class='reverse' onclick='sendRequest(\"/infeed_reverse\")'>Reverse</button>";
    html += "<p> </p>";
    html += "<button onclick='sendRequest(\"/infeed_increase\")'>Speed +</button>";
    html += "<button onclick='sendRequest(\"/infeed_decrease\")'>Speed -</button>";
    html += "</div>";

    // Fan Status and Control Buttons
    html += "<div class='section-box'>";
    html += "<h2>Fan Section</h2>";
    html += "<h3 id='fanStatus'>Fan Status: " + String(getEngineFanState()) + "</h3>";
    html += "<p> Fan automatically engages forward after 1 minute.</p>";
    html += "<button class='forward' onclick='sendRequest(\"/fan_forward\")'>Fan Forward</button>";
    html += "<button class='stop' onclick='sendRequest(\"/fan_stop\")'>Fan Stop</button>";
    html += "<button class='reverse' onclick='sendRequest(\"/fan_reverse\")'>Fan Reverse</button>";
    html += "</div>";

    // JavaScript to handle AJAX
    html += "<script>";
    html += "function sendRequest(url) {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.open('GET', url, true);";
    html += "xhr.send();";
    html += "}";
    
    // Function to load statuses using AJAX
    html += "function loadStatus() {";
    html += "fetchStatus('infeedStatus', '/get_infeed_status');";
    html += "fetchStatus('fanStatus', '/get_fan_status');";
    html += "}";
    
    // Function to update specific element with AJAX response
    html += "function fetchStatus(elementId, url) {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (xhr.readyState == 4 && xhr.status == 200)";
    html += "document.getElementById(elementId).innerHTML = 'Status: ' + xhr.responseText;";
    html += "};";
    html += "xhr.open('GET', url, true);";
    html += "xhr.send();";
    html += "}";

    // setInterval to load statuses every 200ms
    html += "setInterval(loadStatus, 200);";
    html += "</script>";

    html += "</body></html>";
    
    request->send(200, "text/html", html); // Use the request object to send the response
}

void serverHandlers(){
  // The root handler
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      handleRoot(request);
  });

  // Handler for Infeed Section Buttons
  server.on("/infeed_reverse", HTTP_GET, [](AsyncWebServerRequest *request){
      handleInfeedReverseButton(request);
  });

  server.on("/infeed_start", HTTP_GET, [](AsyncWebServerRequest *request){
      handleInfeedStartButton(request);
  });

  server.on("/infeed_stop", HTTP_GET, [](AsyncWebServerRequest *request){
      handleInfeedStopButton(request);
  });

  server.on("/infeed_increase", HTTP_GET, [](AsyncWebServerRequest *request){
      handleInfeedIncreaseButton(request);
  });

  server.on("/infeed_decrease", HTTP_GET, [](AsyncWebServerRequest *request){
      handleInfeedDecreaseButton(request);
  });

  // Handler for Fan Section Buttons
  server.on("/fan_forward", HTTP_GET, [](AsyncWebServerRequest *request){
      handleFanForwardButton(request);
  });

  server.on("/fan_reverse", HTTP_GET, [](AsyncWebServerRequest *request){
      handleFanReverseButton(request);
  });

  server.on("/fan_stop", HTTP_GET, [](AsyncWebServerRequest *request){
      handleFanStopButton(request);
  });

  server.on("/get_infeed_status", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", getInfeedConveyorState());
  });

  server.on("/get_fan_status", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", getEngineFanState());
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200); // placeholder response
  }, handleOTAUpdate);

  server.on("/admin", HTTP_GET, handleAdmin);

}

void handleOTAUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    Serial.printf("OTA Update Start: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }
  if (final) {
    if (Update.end(true)) {
      Serial.printf("OTA Update Success: %u bytes\n", index + len);
      request->send(200, "text/plain", "Update complete! Rebooting...");
    } else {
      Update.printError(Serial);
      request->send(500, "text/plain", "Update failed! Check the logs.");
    }
  }
}


void handleAdmin(AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Admin Page</title>";
    html += "</head><body>";
    html += "<h1>Admin Page</h1>";
    html += "<h2>OTA Update</h2>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update'>";
    html += "<input type='submit' value='Update'>";
    html += "</form>";
    html += "</body></html>";
    
    request->send(200, "text/html", html);
}


String getInfeedConveyorState() {
  switch (infeedConveyorState) {
    case FORWARD: return "FORWARD, Speed: "+ String(infeedSpeed);
    case REVERSE: return "REVERSE, Speed: "+ String(infeedSpeed);
    case NEUTRAL: return "NEUTRAL";
    default: return "UNKNOWN";
  }
}

String getEngineFanState() {
  switch (engineFanState) {
    case FORWARD: return "FORWARD, Speed: "+ String(fanSpeed);
    case REVERSE: return "REVERSE, Speed: "+ String(fanSpeed);
    case NEUTRAL: return "NEUTRAL";
    default: return "UNKNOWN";
  }
}
