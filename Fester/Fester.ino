// Install MD_Parola and MD_MAX72XX
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Configure LED Matrix
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 3 // TODO

MD_Parola m_display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// TODO ADD AMONG US EMOJI, ADD SALUTE

// Messages to Scroll
const char *messages[] =
{
  "GAP BOT",
  "AGENCY",
  "BOT IN THREE WEEKS",
  "THE CLIMBERZ ARE NOT CLIMBING",
  "I WOULD LIKE TO INVITE YOU TO USE YOUR AGENCY",
  "IT'S JUST PROTO",
  "kPhysicsActive = false;",
  "The Grabber will be done in 15 minutes, +/- 20 minutes, Trust",
  "Lets try to give Programming the bot by the end of the meeting",
  "12 volts to data is fine right?",
  "Guys! We are in the 11th hour. We must stay focused.",
  "THE INCENTIVE: RICHMOND RACEWAY TICKETS",
  "We need some Nuts, some Bolts, and some Metal. Oh, dont forget Wheels"
};
const char* sep = "   ";

const char *message;

void setup() {
  int size = 0;

  // Determine the total size needed for the concatenated message
  for (int i = 0; i < sizeof(messages)/sizeof(*messages); i++) {
    size += strlen(messages[i]);
    if (i < sizeof(messages)/sizeof(*messages) - 1) {
      size += strlen(sep);
    }
  }

  // Allocate memory for the concatenated message
  message = new char[size + 1];

  // Concatenate the messages with the separator
  strcpy(message, messages[0]);
  for (int i = 1; i < sizeof(messages)/sizeof(*messages); i++) {
    strcat(message, sep);
    strcat(message, messages[i]);
  }

  // Setup the LED Display
  m_display.begin();
  m_display.displayScroll(message, PA_CENTER, PA_SCROLL_LEFT, 50);
}

void loop() {
  if(m_display.displayAnimate()) {
    m_display.displayReset();
  }
}
