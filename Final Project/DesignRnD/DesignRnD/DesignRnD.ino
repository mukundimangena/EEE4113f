const int ledPins[] = {13, 27, 18, 19};
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);
const int audio[] = {14};
const int numAudio = sizeof(audio) / sizeof(audio[0]);
const int sirenfreq[] = {2000, 5000};
const int numfreq = sizeof(sirenfreq) / sizeof(sirenfreq[0]);
int PatternArray[] = {1, 2};
int ParraySize = sizeof(PatternArray) / sizeof(PatternArray[0]);
const int PWMfreq = 5000;
const int ledcResolution = 8;
const int ledcDutyCycle = (1 << (ledcResolution - 1));
int receivedValue = 0;
unsigned long lastToggleTimes[10];
unsigned long blinkDurations[10];
bool ledStates[10];

// GPIO21 trigger pin
const int triggerPin = 21;
bool lastTriggerState = HIGH;
bool currentTriggerState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce

// State variables
int currentDutyCycle = 255;
unsigned long lastUpdate = 0;
bool fadingPhase = true;
bool pwmActive = false;

void setup() {
  Serial.begin(9600);
  Serial.println("System ready - GPIO21 trigger active");

  // Setup trigger pin with internal pullup resistor
  pinMode(triggerPin, INPUT_PULLUP);

  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
    lastToggleTimes[i] = millis();
    blinkDurations[i] = random(100, 1000);
    ledStates[i] = false;
  }
  for (int i = 0; i < numAudio; i++) {
    pinMode(audio[i], OUTPUT);
  }
  randomSeed(analogRead(0));
}

bool readTriggerPin() {
  bool reading = digitalRead(triggerPin);
  
  // Check if the state has changed (debounce logic)
  if (reading != lastTriggerState) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed since the last state change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the state has actually changed
    if (reading != currentTriggerState) {
      currentTriggerState = reading;
      lastTriggerState = reading;
      
      // Return true on falling edge (button press, since we use pullup)
      if (currentTriggerState == LOW) {
        return true;
      }
    }
  }
  
  lastTriggerState = reading;
  return false;
}

void PWMStart() {
  if (!pwmActive) {
    currentDutyCycle = 255;
    fadingPhase = true;
    pwmActive = true;
    lastUpdate = millis();
    // Attach ALL LEDs at once
    for (int i = 0; i < numLeds; i++) {
      ledcAttachChannel(ledPins[i], PWMfreq, ledcResolution, i);
    }
  }
}

bool PWMUpdate() {
  if (!pwmActive || millis() - lastUpdate < (fadingPhase ? 15 : 100)) {
    return false;
  }
  
  lastUpdate = millis();
  
  if (fadingPhase) {
    // Fade ALL LEDs together
    for (int i = 0; i < numLeds; i++) {
      ledcWrite(ledPins[i], currentDutyCycle);
    }
    currentDutyCycle--;
    if (currentDutyCycle < 0) {
      // Switch ALL to digital HIGH
      for (int i = 0; i < numLeds; i++) {
        ledcDetach(ledPins[i]);
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], HIGH);
      }
      fadingPhase = false;
    }
  } else {
    // Turn ALL LEDs OFF and complete
    for (int i = 0; i < numLeds; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    pwmActive = false;
    return true;
  }
  
  return false;
}

void randomLionLightsConcurrent() {
  unsigned long currentTime = millis();
  for (int i = 0; i < numLeds; i++) {
    if (currentTime - lastToggleTimes[i] >= blinkDurations[i]) {
      ledStates[i] = !ledStates[i];
      digitalWrite(ledPins[i], ledStates[i] ? HIGH : LOW);
      blinkDurations[i] = random(100, 1000);
      lastToggleTimes[i] = currentTime;
    }
  }
}

void acceleratingBlinkPatternNonBlocking(int startDelay, int endDelay, int stepDelay) {
  static unsigned long lastTime = 0;
  static int currentDelay = startDelay;
  static bool patternState = false;
  static int fastBlinkCount = 0;
  static bool inFastBlink = false;
  static bool initialized = false;
  static int savedStartDelay = 0; // Store original start delay

  // Initialize only once or when parameters change
  if (!initialized || startDelay != savedStartDelay) {
    currentDelay = startDelay;
    savedStartDelay = startDelay;
    patternState = false;
    fastBlinkCount = 0;
    inFastBlink = false;
    initialized = true;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= (inFastBlink ? endDelay : currentDelay)) {
    lastTime = currentTime;
    patternState = !patternState;
    
    // First half of LEDs (LED1, LED2)
    for (int i = 0; i < numLeds / 2; i++) {
      digitalWrite(ledPins[i], patternState ? HIGH : LOW);
    }
    
    // Second half of LEDs (LED3, LED4) - opposite state
    for (int i = numLeds / 2; i < numLeds; i++) {
      digitalWrite(ledPins[i], patternState ? LOW : HIGH);
    }
    
    if (!inFastBlink) {
      currentDelay -= stepDelay;
      if (currentDelay <= endDelay) {
        inFastBlink = true;
        currentDelay = endDelay;
        fastBlinkCount = 0;
      }
    } else {
      fastBlinkCount++;
      if (fastBlinkCount >= 10) { // Increased from 5 to 10 for more fast blinks
        currentDelay = startDelay;
        inFastBlink = false;
        fastBlinkCount = 0;
        // Don't reset initialized here - let it continue running
      }
    }
  }
}

void updateSiren(int delayTime) {
  static unsigned long lastTime = 0;
  static bool highTone = false;
  static bool sirenAttached = false;
  
  if (millis() - lastTime >= delayTime) {
    lastTime = millis();
    if (!sirenAttached) {
      ledcAttachChannel(audio[0], 2000, ledcResolution, 5);
      sirenAttached = true;
    }
    highTone = !highTone;
    ledcChangeFrequency(audio[0], highTone ? 5000 : 2000, ledcResolution);
    ledcWrite(audio[0], ledcDutyCycle);
  }
}

void BuzzerSweep(int minFreq, int maxFreq, int stepSize, int delayMs) {
  static unsigned long lastTime = 0;
  static int freq = minFreq;
  static int direction = stepSize;
  static bool attached = false;
  static int lastMinFreq = 0, lastMaxFreq = 0, lastStep = 0;
  
  if (minFreq != lastMinFreq || maxFreq != lastMaxFreq || stepSize != lastStep) {
    freq = minFreq;
    direction = stepSize;
    lastMinFreq = minFreq;
    lastMaxFreq = maxFreq;
    lastStep = stepSize;
  }
  
  if (millis() - lastTime >= delayMs) {
    lastTime = millis();
    if (!attached) {
      ledcAttachChannel(audio[0], freq, ledcResolution, 5);
      attached = true;
    }
    freq += direction;
    if (freq >= maxFreq || freq <= minFreq) direction = -direction;
    ledcChangeFrequency(audio[0], freq, ledcResolution);
    ledcWrite(audio[0], ledcDutyCycle);
  }
}

void blinkSequenceNonBlocking(int delayTime) {
  static unsigned long lastTime = 0;
  static int blinkCount = 1;
  static int currentBlink = 0;
  static bool ledOn = false;
  static bool inPause = false;
  static bool initialized = false;

  if (!initialized) {
    blinkCount = 1;
    currentBlink = 0;
    ledOn = false;
    inPause = false;
    initialized = true;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= (inPause ? 500 : delayTime)) {
    lastTime = currentTime;

    if (inPause) {
      inPause = false;
      blinkCount++;
      currentBlink = 0;
      if (blinkCount > 3) {
        blinkCount = 1;
        initialized = false; // Reset for next call
        return;
      }
    } else {
      ledOn = !ledOn;
      for (int j = 0; j < numLeds; j++) {
        digitalWrite(ledPins[j], ledOn ? HIGH : LOW);
      }
      if (!ledOn) {
        currentBlink++;
        if (currentBlink >= blinkCount) {
          inPause = true;
        }
      }
    }
  }
}

int selectPattern() {
  // Option 1: Cycle through patterns (0, 1, 0, 1, ...)
  // static int lastPattern = -1;
  // lastPattern = (lastPattern + 1) % 2; // Alternate between 0 and 1
  // return lastPattern;

  // Option 2: Randomly select a pattern
  return random(0, 4); // Randomly choose 0 or 1
}

void loop() {
  static bool pwmRequested = false;
  static int lastReceivedValue = 0;
  static int currentPattern = 0; // Track the selected pattern (0 or 1)
  static bool systemActive = false; // Track if system is currently active

  // Check for GPIO21 trigger
  if (readTriggerPin()) {
    if (!systemActive) {
      // Turn system ON - equivalent to entering "1"
      pwmRequested = true;
      systemActive = true;
      currentPattern = selectPattern();
      Serial.println("GPIO21 triggered - Running pattern " + String(currentPattern + 1));
    } else {
      // Turn system OFF - equivalent to entering "0"
      ledcWrite(audio[0], 0);
      for (int i = 0; i < numLeds; i++) {
        digitalWrite(ledPins[i], LOW);
      }
      systemActive = false;
      pwmActive = false;
      pwmRequested = false;
      Serial.println("GPIO21 triggered - All systems OFF");
    }
  }

  // Execute the selected pattern if system is active
  if (systemActive) {
    // Handle PWM start if requested
    if (pwmRequested && !pwmActive) {
      PWMStart();
    }

    // Continue with pattern execution based on currentPattern
    if (currentPattern == 0) {
      // Pattern 0 logic - add your specific pattern here
      randomLionLightsConcurrent();
      updateSiren(100);
    } else if (currentPattern == 1) {
      // Pattern 1 logic - add your specific pattern here
      acceleratingBlinkPatternNonBlocking(500, 50, 25);
      BuzzerSweep(1000, 3000, 50, 10);
    } else if (currentPattern == 2) {
      // Pattern 2 logic
      blinkSequenceNonBlocking(200);
      updateSiren(200);
    } else if (currentPattern == 3) {
      // Pattern 3 logic
      acceleratingBlinkPatternNonBlocking(300, 75, 15);
      BuzzerSweep(500, 4000, 100, 5);
    }

    // Update PWM if active
    if (PWMUpdate()) {
      pwmRequested = false;
    }
  }
}