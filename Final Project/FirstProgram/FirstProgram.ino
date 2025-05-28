// Define the buzzer pin and LEDC channel
#define BUZZER_PIN 14
#define LEDC_CHANNEL 5
#define LEDC_FREQ 2400

void setup() {
  // Attach the buzzer pin to the LEDC channel
  ledcAttachChannel(BUZZER_PIN, 2400, 8, LEDC_CHANNEL);
  // Configure the LEDC channel with frequency and resolution
  ledcWriteTone(14, LEDC_FREQ); // Set frequency to 2400 Hz
}

void loop() {
  // Turn buzzer on (50% duty cycle for medium volume)
  ledcWrite(14, 128); // 8-bit resolution, 128 = ~50% duty
  delay(1000); // Buzz for 1 second
  
  // Turn buzzer off
  ledcWrite(14, 0);
  delay(1000); // Pause for 1 second
}
