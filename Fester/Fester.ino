// Install MD_Parola and MD_MAX72XX
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Configure LED Matrix
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 0 // TODO

MD_Parola m_display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Messages to Scroll
const char *message = "GAP BOT   AGENCY   BOT IN THREE WEEKS";

void setup() {
  // Setup the LED Display
  m_display.begin();
  m_display.displayScroll(message, PA_CENTER, PA_SCROLL_RIGHT, 100);
}

void loop() {
  if(m_display.displayAnimate()) {
    m_display.displayReset();
  }
}
