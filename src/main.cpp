#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
//#include <Servo.h>
#include <ESP32Servo.h>
const char* WIFI_SSID    = "AP RE";
const char* WIFI_PASS = "eiramsor44wl";

#define LED_PIN   26
#define BTN_PIN   22

#define HTTP_PORT 80

AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");

//pin mode for motor R
int motor1EnablePin = 14;
int motor1pin1 = 27;
int motor1pin2 = 26;

//pin mode for motor L
int motor2EnablePin = 32;
int motor2pin1 = 33;
int motor2pin2 = 25;

//motor PWM channel must be set 2 and 3, rather than 0.
const int freq = 30000;
const int motor1pwmChannel = 2;
const int motor2pwmChannel = 3;
const int resolution = 8;
int dutyCycle = 0; //for speed control


//servo
Servo steeringServo;
int steeringServoPin = 18;

// ----------------------------------------------------------------------------
// Definition of the LED component
// ----------------------------------------------------------------------------

struct Led {
    // state variables
    uint8_t pin;
    bool    on;

    // methods
    void update() {
        digitalWrite(pin, on ? HIGH : LOW);
    }
};

// ----------------------------------------------------------------------------
// Definition of the Button component
// ----------------------------------------------------------------------------
// Button debouncing
const uint8_t DEBOUNCE_DELAY = 10; // in milliseconds

struct Button {
    // state variables
    uint8_t  pin;
    bool     lastReading;
    uint32_t lastDebounceTime;
    uint16_t state;

    // methods determining the logical state of the button
    bool pressed()                { return state == 1; }
    bool released()               { return state == 0xffff; }
    bool held(uint16_t count = 0) { return state > 1 + count && state < 0xffff; }

    // method for reading the physical state of the button
    void read() {
        // reads the voltage on the pin connected to the button
        bool reading = digitalRead(pin);

        // if the logic level has changed since the last reading,
        // we reset the timer which counts down the necessary time
        // beyond which we can consider that the bouncing effect
        // has passed.
        if (reading != lastReading) {
            lastDebounceTime = millis();
        }

        // from the moment we're out of the bouncing phase
        // the actual status of the button can be determined
        if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
            // don't forget that the read pin is pulled-up
            bool pressed = reading == LOW;
            if (pressed) {
                     if (state  < 0xfffe) state++;
                else if (state == 0xfffe) state = 2;
            } else if (state) {
                state = state == 0xffff ? 0 : 0xffff;
            }
        }

        // finally, each new reading is saved
        lastReading = reading;
    }
};

Led    onboard_led = { LED_BUILTIN, false };
Led    led         = { LED_PIN, false };
Button button      = { BTN_PIN, HIGH, 0, 0 };


// *************
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    /*
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

        const uint8_t size = JSON_OBJECT_SIZE(1);
        StaticJsonDocument<size> json;
        DeserializationError err = deserializeJson(json, data);
        if (err) {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.c_str());
            return;
        }

        const char *action = json["action"];
        if (strcmp(action, "toggle") == 0) {
            led.on = !led.on;
            notifyClients();
        }

    }
    */
}

void onEvent(AsyncWebSocket       *server,
             AsyncWebSocketClient *client,
             AwsEventType          type,
             void                 *arg,
             uint8_t              *data,
             size_t                len) {

    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void initWebSocket() {
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

///motor motion
void moveForward(){
  //motor R
  digitalWrite(motor1pin1, LOW); 
  digitalWrite(motor1pin2, HIGH);
  //motor L
  digitalWrite(motor2pin1, LOW); 
  digitalWrite(motor2pin2, HIGH); 

}

void moveBackward(){
  //motor R
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  //motor L
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void moveStop(){
  //motor R
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  //motor L
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);  
}

void FrontAndBackMoveDirectionControl(String directionData){
      //control forward and backward move///
     //convert string to int
    uint8_t Direction = directionData.toInt();
    //map it to be 0-100, and then convert to string
       
    if(Direction > 31){
      moveBackward();
      /// map motor speed from 140 (motor stop) to 255 
      dutyCycle = map(Direction, 31,60,170,255);
      ledcWrite(motor1pwmChannel, dutyCycle);
      ledcWrite(motor2pwmChannel, dutyCycle);
      Serial.print("move backward ->");
      Serial.println(dutyCycle);
    }
    else if(Direction < 29){
      moveForward();
      /// map motor speed from 140 (motor stop) to 255 
      dutyCycle = map(Direction, 29,0,170,255);
      ledcWrite(motor1pwmChannel, dutyCycle);
      ledcWrite(motor2pwmChannel, dutyCycle);
       Serial.print("move forward ->");
       Serial.println(dutyCycle);

    }

    else{
      Serial.println("move stop");
        moveStop();
    }
       

    
}
int steeringAngle = 0;
void steeringDirectionControl(String SteeringData){
  //control servo motor to steering//
  Serial.print("servo direction");
  Serial.println(SteeringData);
  int Steering = SteeringData.toInt();
 
     if(Steering > 30){
      Serial.println("move right");
      Serial.println(90+Steering-15);//my servo motor place offset...I made the steering angle trickly..steering angle is 55 to 135
      steeringAngle = map(Steering, 30,60,90,135);
      steeringServo.write(steeringAngle);

    }
    else if(Steering < 30){
      Serial.println("move left");

      Serial.println(90-(35-Steering));//my servo motor place offset...I made the steering angle trickly..steering angle is 55 to 135
      steeringAngle = map(Steering, 30,0,90,55);
      steeringServo.write(steeringAngle);

    } 
}




// *************


///response from web socket
void webSocketEvent(uint8_t num, int type, uint8_t*payload, size_t length){
    Serial.printf("webSocketEvent(%d,%d,...)\r\n", num, type);
    switch(type){
      case WS_EVT_DISCONNECT:
        Serial.printf("[%u]Disconnected!\r\n",num);
        break;
      case WS_EVT_CONNECT:{
        //IPAddress ip = webSocket.remoteIP(num);
        //Serial.printf("[%u]Connected from %d.%d.%d.%d url:%s\r\n",num,ip[0],ip[1],ip[2],ip[3],payload);
        

      }
        break;
      case WS_EVT_DATA:
        Serial.printf("[%u]get Text: %s\r\n",num,payload);
        //Serial.printf((const char*) payload);
        //strcmp compare 2 strings (need to char*str)
        //if Return value < 0 then it indicates str1 is less than str2.
        //if Return value > 0 then it indicates str2 is less than str1.
        //if Return value = 0 then it indicates str1 is equal to str2.
        if(payload[0] == 'F'){
          //convert to String from payload
            String strPayload = String((const char*) payload);
            //remove 1st char from string
            strPayload.remove(0,1);
          //  Serial.println(strPayload);
          //function to control motors for move forward and backward
          FrontAndBackMoveDirectionControl(strPayload);
           
        }

        if(payload[0] == 'L'){
          //convert to String from payload
            String strPayload = String((const char*) payload);
            //remove 1st char from string
            strPayload.remove(0,1);
            //Serial.println(strPayload);
            //function to control servo motor to rotate left and right 
            steeringDirectionControl(strPayload);
        }

        //send all data to all connected clients/browser
       // webSocket.broadcastTXT(payload, length);
    
        break;
        /*
      case WS_EVT_BINARY:
        Serial.printf("[%u]get binary length: %u\r\n",num,length);      
        break;
        */
    }   
}


// ----------------------------------------------------------------------------
// SPIFFS initialization
// ----------------------------------------------------------------------------

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("Cannot mount SPIFFS volume...");
    while (1) {
        onboard_led.on = millis() % 200 < 50;
        onboard_led.update();
    }
  }
}

// ----------------------------------------------------------------------------
// Connecting to the WiFi network
// ----------------------------------------------------------------------------

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Trying to connect [%s] ", WiFi.macAddress().c_str());
  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
  }
  Serial.printf(" success:  %s\n", WiFi.localIP().toString().c_str());
}

String processor(const String &var) {
    return String(var == "STATE" && led.on ? "on" : "off");
}


void onRootRequest(AsyncWebServerRequest *request) {
  request->send(SPIFFS, "/index.html", "text/html", false, processor);
}

void initWebServer() {
    server.on("/", onRootRequest);
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
}

void notifyClients() 
{
  Serial.printf("notifyClients");
  return;
    const uint8_t size = JSON_OBJECT_SIZE(1);
    StaticJsonDocument<size> json;
    json["status"] = led.on ? "on" : "off";

    char buffer[17];
    size_t len = serializeJson(json, buffer);
    ws.textAll(buffer, len);
}

void setup() 
{
Serial.begin(115200);
    pinMode(onboard_led.pin, OUTPUT);
    pinMode(led.pin,         OUTPUT);
    pinMode(button.pin,      INPUT);

    Serial.begin(115200); delay(500);

    initSPIFFS();
    initWiFi();
    initWebSocket();
    initWebServer();


   //set up servo
   steeringServo.attach(steeringServoPin);
   //directionServo.write(90);

  //set up motors
  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor1pin1, OUTPUT); 
  pinMode(motor1pin2, OUTPUT);  

  pinMode(motor2EnablePin, OUTPUT);
  pinMode(motor2pin1, OUTPUT); 
  pinMode(motor2pin2, OUTPUT); 
  
  
  //configure PWM channel 
  ledcSetup(motor1pwmChannel,freq, resolution);
  ledcSetup(motor2pwmChannel,freq, resolution);
  
  // Attach the PWM channel to the enable pins which are the GPIOs to be controlled
  ledcAttachPin(motor1EnablePin, motor1pwmChannel);
  ledcAttachPin(motor2EnablePin, motor2pwmChannel);
  
  // Produce a PWM signal to both enable pins with a duty cycle 0
  ledcWrite(motor1pwmChannel, dutyCycle);
  ledcWrite(motor2pwmChannel, dutyCycle);  
  
  
  }

void loop() 
{
 ws.cleanupClients();
   button.read();

    if (button.pressed()) {
        led.on = !led.on;
        notifyClients();
    }
    
    onboard_led.on = millis() % 1000 < 50;

    led.update();
    onboard_led.update();


}