  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <ArduinoJson.h>
  #include <FirebaseClient.h>
  #include <WiFiClientSecure.h>
  #define TINY_GSM_MODEM_SIM800 
  #include <TinyGsmClient.h>   
  #include <HardwareSerial.h> 
  #include <Adafruit_Fingerprint.h> 

  const char* ssidList[] = {
  };

  const char* passwordList[] = {
  };

  #define SerialMon Serial

  // Relay pin for solenoid lock
  #define RELAY_PIN 23

  //SIM800L pins
  #define MODEM_TX 22
  #define MODEM_RX 21

  //AS608 pins
  #define FINGERPRINT_RX 16  
  #define FINGERPRINT_TX 17  

  //Serials for SIM and FingerPrint
  #define SerialAT Serial1   // SIM800 Serial (using pins 21 and 22)
  #define SerialFP Serial2   // Fingerprint Serial (using pins 16 and 17)

  //SIM800 Materials
  TinyGsm modem(SerialAT);
  Adafruit_Fingerprint finger = Adafruit_Fingerprint(&SerialFP);

  const String deviceStatusUrl = "";
  const String mobileNoUrl = "";
  //Firebase Firestore materials
  #define API_KEY ""  //api key from firebase
  #define USER_EMAIL "" //this is for authentication
  #define USER_PASSWORD ""
  #define FIREBASE_PROJECT_ID "" 
  void printResult(AsyncResult &aResult);
  DefaultNetwork network;
  UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
  FirebaseApp app;
  WiFiClientSecure ssl_client;
  using AsyncClient = AsyncClientClass;
  AsyncClient aClient(ssl_client, getNetwork(network));
  Firestore::Documents Docs; // this is for the firestore
  AsyncResult aResult_no_callback; 

  int counter = 0;
  bool isLocked = true;
  bool isUpdatingLock = false; // New flag to track if we're in the process of locking/unlocking
  int fpAttempts = 0; //finger print attempts
  bool taskCompleted = false;
  unsigned long lastFingerprintCheck = 0;
  const unsigned long fingerprintInterval = 3000;  // Interval of 3 seconds
  enum LockState { IDLE, UNLOCKED, LOCKING };
  LockState currentState = IDLE;
  unsigned long lockTime = 0;  // Store the time when the lock was unlocked
  const unsigned long lockDelay = 5000;  // 5 seconds delay before locking

  void setup() {
      Serial.begin(115200);
      connectToWiFi();
      SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);  
      SerialFP.begin(57600, SERIAL_8N1, FINGERPRINT_RX, FINGERPRINT_TX);
      finger.begin(57600);

      if (finger.verifyPassword()) {
          Serial.println("Fingerprint sensor found!");
      } else {
          Serial.println("Fingerprint sensor not detected :(");
      }

      
      Serial.println("Initializing modem...");
      modem.restart();

      Serial.println("Waiting for network...");
    
    // Wait for the modem to connect to the network
      if (!modem.waitForNetwork()) {
          Serial.println("Failed to connect to network.");
      } else {
          Serial.println("Connected to network!");
      }
      // Print signal quality
      int signalQuality = modem.getSignalQuality();
      Serial.print("Signal Quality: ");
      Serial.println(signalQuality);


      pinMode(RELAY_PIN, OUTPUT);
      //testing unlock and lock the solenoid 
      digitalWrite(RELAY_PIN, LOW);
      delay(3000);
      digitalWrite(RELAY_PIN, HIGH);
      
      ssl_client.setInsecure();
      initializeApp(aClient, app, getAuth(user_auth), aResult_no_callback);
      app.getApp<Firestore::Documents>(Docs);
  }

 void loop() {
    app.loop();
    Docs.loop();

    // Check if enough time has passed since the last fingerprint check
    if (app.ready() && millis() - lastFingerprintCheck >= fingerprintInterval) {
        lastFingerprintCheck = millis();  // Update the last check time

        if (!isUpdatingLock) {  // Only check fingerprint if not in the middle of an update
            Serial.println("Waiting for fingerprint...");
            int fingerprintID = getFingerprintID();

            if (fingerprintID == -2) {  // No finger detected
                Serial.println("No finger detected on sensor");
            } 
            else if (fingerprintID != -1) {  // Valid fingerprint detected
                fpAttempts = 0;  // Reset attempts when fingerprint is found
                Serial.println("Fingerprint ID " + String(fingerprintID) + " found");
                delay(2000);

                // Only unlock if currently idle (to avoid repeated unlocks)
                if (currentState == IDLE) {
                    // Update Firebase Firestore to set the lock status to unlocked
                    isLocked = false;
                    updateFirebaseLockStatus(isLocked);
                    // Unlock the solenoid lock
                    digitalWrite(RELAY_PIN, LOW);
                    Serial.println("Lock unlocked");

                    // Set flag to indicate we're updating the lock state
                    isUpdatingLock = true;

                    // Transition to UNLOCKED state
                    currentState = UNLOCKED;
                    lockTime = millis();  // Record the time the lock was unlocked
                }
            } 
            else if (fingerprintID == -1) {  // Fingerprint detected but no match
                Serial.println("Fingerprint scan failed.");
                fpAttempts++;  // Increment attempts only on explicit failure
                delay(5000);
            }
        }
    }

    // Manage lock state transitions
    if (currentState == UNLOCKED && millis() - lockTime >= lockDelay) {
        // 5 seconds have passed, lock the solenoid and update Firebase Firestore again
        isLocked = true;
        updateFirebaseLockStatus(isLocked);

        // Lock the solenoid
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("Lock locked");

        // Transition to LOCKING state
        currentState = LOCKING;

        // Clear the flag after the lock is updated
        isUpdatingLock = false;
    }

    // Transition back to IDLE state once locking is done
    if (currentState == LOCKING) {
        currentState = IDLE;
    }

    // If attempts reach 3 or more, alert about potential intruder
    if (fpAttempts == 3) {
        fetchMobileNumbersAndSendSMS(true);
        fpAttempts = 0;  // Reset after logging the message
    }

    // Fetch device status only if not updating the lock manually
    if (!isUpdatingLock) {
        fetchDeviceStatus();
    }
}

int getFingerprintID() {
    uint8_t p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) {  // Check if no finger is placed
        return -2;  // Special code to indicate no finger detected
    } 
    else if (p != FINGERPRINT_OK) {
        Serial.println("Failed to capture fingerprint image.");
        return -1;
    }

    p = finger.image2Tz();
    if (p != FINGERPRINT_OK) {
        Serial.println("Failed to convert image to template.");
        return -1;
    }

    p = finger.fingerFastSearch();  // Fast search for fingerprint match
    if (p == FINGERPRINT_OK) {
        Serial.print("Match found! ID: ");
        Serial.println(finger.fingerID);  // Output the matched fingerprint ID
        return finger.fingerID;  // Return the matched ID
    } 
    else if (p == FINGERPRINT_NOTFOUND) {
        Serial.println("No match found.");
        return -1;
    } 
    else {
        Serial.println("Error during fingerprint search.");
        return -1;
    }
}


  void updateFirebaseLockStatus(bool status) {
      String devicePath = "device/WuEC8rfVaZLOuSrp8p33";
      Values::BooleanValue boolV(status);
   
      Document<Values::Value> doc("isLocked", Values::Value(boolV));
      
      PatchDocumentOptions patchOptions(DocumentMask("isLocked"), DocumentMask(), Precondition());
      Docs.patch(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), devicePath, patchOptions, doc, aResult_no_callback);
  }

  void fetchDeviceStatus() {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(deviceStatusUrl);  // Use the Firestore API URL for the device status

      int httpCode = http.GET();  // Send the request

      if (httpCode > 0) {  // HTTP request successful
        String payload = http.getString();

        // Parse JSON response from Firestore
        DynamicJsonDocument doc(1024);  // Adjust size if necessary
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
          Serial.print("Failed to parse JSON: ");
          Serial.println(error.c_str());
          return;
        }

        // Extract the `isLocked` field from the JSON
        bool isLocked = doc["fields"]["isLocked"]["booleanValue"].as<bool>();

        // Control solenoid lock based on the isLocked value
        if (isLocked) {
          // Lock the solenoid (deactivate relay)
          digitalWrite(RELAY_PIN, HIGH);  // Set relay pin to HIGH (lock)
          Serial.println("Solenoid Locked");
        } else {
          // Unlock the solenoid (activate relay)
          digitalWrite(RELAY_PIN, LOW);  // Set relay pin to LOW (unlock)
          Serial.println("Solenoid Unlocked");
        }
      } else {
        Serial.print("Error in HTTP request: ");
        Serial.println(httpCode);
      }

      http.end();  // Close connection
    } else {
      Serial.println("WiFi Disconnected");
    }
  }

 
  void connectToWiFi() {
      for (int i = 0; i < sizeof(ssidList) / sizeof(ssidList[0]); i++) {
          WiFi.begin(ssidList[i], passwordList[i]);
          Serial.print("Connecting to Wi-Fi: ");
          Serial.println(ssidList[i]);

          unsigned long startAttemptTime = millis(); // Record the start time

          // Wait for connection or timeout after 1 minute (60000 milliseconds)
          while (WiFi.status() != WL_CONNECTED) {
              if (millis() - startAttemptTime >= 10000) { // 1 minute timeout
                  Serial.println("Connection timed out, moving to next Wi-Fi...");
                  break; // Exit the loop if timeout occurs
              }
              delay(300);
              Serial.print(".");
          }

          if (WiFi.status() == WL_CONNECTED) {
              Serial.println("Connected with IP: " + WiFi.localIP().toString());
              return; // Exit after successful connection
          }
      }

      Serial.println("Failed to connect to any Wi-Fi network.");
  }

  void fetchMobileNumbersAndSendSMS(bool isAlert) {
      if (WiFi.status() == WL_CONNECTED) {
          HTTPClient http;
          http.begin(mobileNoUrl);  // Firestore API URL for users with mobile numbers

          int httpCode = http.GET();  // Send the request

          if (httpCode > 0) {  // HTTP request was successful
              String payload = http.getString();
              Serial.println(payload);
              // Parse JSON response from Firestore
              DynamicJsonDocument doc(4096);  // Adjust size if necessary based on the payload
              DeserializationError error = deserializeJson(doc, payload);

              if (error) {
                  Serial.print("Failed to parse JSON: ");
                  Serial.println(error.c_str());
                  return;
              }

              // Iterate through the documents (user entries)
              JsonArray usersArray = doc["documents"].as<JsonArray>();

              for (JsonVariant user : usersArray) {
                  // Get the mobile number from each user's fields
                  String mobileNo = user["fields"]["mobileNumber"]["stringValue"].as<String>();

                  // Send SMS to the mobile number
                  if (!mobileNo.isEmpty()) {
                      sendSMSToSIM800L(mobileNo, isAlert);  // Send SMS to each mobile number
                  } else {
                      Serial.println("Mobile number missing for user.");
                  }
              }
          } else {
              Serial.print("Error in HTTP request: ");
              Serial.println(httpCode);
          }

          http.end();  // Close connection
      } else {
          Serial.println("WiFi Disconnected");
      }
  }

  void sendSMSToSIM800L(String mobileNo, bool isAlert) {
      Serial.println("Sending SMS...");
      if (isAlert) {
          // If the alert flag is true, send an alert SMS
          if (modem.sendSMS(mobileNo, "UNLOCK ME: Someone's trying to unlock your door!")) {
              Serial.println("Alert SMS sent successfully to " + mobileNo);
          } else {
              Serial.println("Failed to send alert SMS to " + mobileNo);
          }
  }

}
