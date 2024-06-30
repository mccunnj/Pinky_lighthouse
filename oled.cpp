#include "oled.h"

Adafruit_SSD1306 oled(128, 32, &Wire);

// Scrolling variables
bool scrolling = false;
int16_t scrollOffset = 0;
unsigned long lastScrollTime = 0;
const int scrollSpeed = 10; // Adjust scrolling speed (milliseconds between updates)
const char* scrollMessage = nullptr;
uint8_t scrollTextSize = 4;
int16_t scrollLength = 0;


void setupOLED() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {
    for (;;) {
      Serial.println("display init failed");
    }
  } else {
    oled.cp437(true);
    oled.clearDisplay();
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.setTextSize(1);
    oled.print(ESP.getChipModel());
    oled.display();
  }
}

void printSmall( const char* message) {
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.print(message);
  oled.display();
}

void printFit(const char* message) {
  // Serial.println(message);
  oled.clearDisplay();
  oled.stopscroll();
  oled.setTextColor(WHITE);
  oled.invertDisplay(false);
  // Maximum text size
  uint8_t textSize = 4;

  //Set scrolling
  scrolling = false;

  // Set the initial cursor position
  int16_t x = 0;
  int16_t y = 0;

  // Print the message with the largest text size
  oled.setTextSize(textSize);

  // Measure the dimensions of the message
  uint16_t textWidth, textHeight;  // Use uint16_t for width and height

  const char* rfMessage = reformatMessage(message, textSize);
  oled.getTextBounds(rfMessage, 0, 0, &x, &y, &textWidth, &textHeight);
  
  // Check if the message fits within the display width
  while (textWidth > oled.width() || textHeight > oled.height()) {
    // Reduce text size if it's too wide
    textSize--;
    if (textSize < 1) {
      scrolling = true;
      break;  // Avoid negative or zero text size
    }
    oled.setTextSize(textSize);
    rfMessage = reformatMessage(message, textSize);
    oled.getTextBounds(rfMessage, 0, 0, &x, &y, &textWidth, &textHeight);
  }
  // Serial.print("textWidth: ");
  // Serial.println(textWidth);
  // Serial.print("textHeight: ");
  // Serial.println(textHeight);
  // Serial.print("textSize: ");
  // Serial.println(textSize);
  if (!scrolling && textHeight <= 32){
    // Calculate the vertical offset to center the entire text block
    int16_t yOffset = (oled.height() - textHeight) / 2;

    // Split the message into lines
    char* lines[10];
    int lineCount = 0;
    char* token = strtok(const_cast<char*>(rfMessage), "\n");
    while (token != NULL && lineCount < 10) {
        lines[lineCount++] = token;
        token = strtok(NULL, "\n");
    }

    // Print each line centered horizontally and adjust y position for each line
    for (int i = 0; i < lineCount; i++) {
        // Measure the width of the current line
        uint16_t lineWidth, lineHeight;
        oled.getTextBounds(lines[i], 0, 0, &x, &y, &lineWidth, &lineHeight);

        // Calculate x position to center the line horizontally
        x = (oled.width() - lineWidth) / 2;

        // Print the line at the calculated position
        oled.setCursor(x, yOffset);
        oled.print(lines[i]);
        // Serial.print("Line: ");
        // Serial.println(lines[i]);

        // Adjust yOffset for the next line
        yOffset += lineHeight;
    }

    oled.display(); // Display the entire text block
  }else {
    // Set up scrolling
    // Serial.print("Setting up scrolling");
    scrollMessage = message;
    //scrollTextSize = textSize;
    scrollOffset = 0;
    // Calculate scroll length
    int messageLength = strlen(message);
    scrollLength = (messageLength * 24) - oled.width() + 24; // 24 pixels per character at text size 4
    scrolling = true;
    lastScrollTime = millis();
  }
}

// Function to reformat message based on current text size
const char* reformatMessage(const char* originalMessage, uint8_t textSize) {
    static char formattedMessage[255]; // Adjust size as needed
    strncpy(formattedMessage, originalMessage, sizeof(formattedMessage));

    // Calculate max line length based on current text size and display width
    uint16_t maxLineLength = oled.width() / (6 * textSize); // Adjust based on actual character width and textSize

    // Track where to insert newlines
    uint16_t lastSpaceIndex = 0;
    uint16_t lineLength = 0;

    // Iterate through message to insert line breaks
    for (uint16_t i = 0; i < strlen(formattedMessage); i++) {
        if (formattedMessage[i] == ' ') {
            lastSpaceIndex = i; // Update last space position
        }
        lineLength++;

        // Check if we need to insert a newline
        if (lineLength >= maxLineLength) {
            if (lastSpaceIndex > 0) {
                formattedMessage[lastSpaceIndex] = '\n'; // Insert newline at last space
                lineLength = i - lastSpaceIndex; // Reset line length
                lastSpaceIndex = 0; // Reset last space index
            }
        }
    }

    return formattedMessage;
}
void updateScroll() {
    if (scrolling && (millis() - lastScrollTime >= scrollSpeed)) {
        lastScrollTime = millis();
        //Serial.print("updatingScroll");
        oled.clearDisplay();
        oled.setTextSize(scrollTextSize);
        oled.setCursor(-scrollOffset, 0);
        oled.print(scrollMessage);
        oled.display();

        scrollOffset++;
        if (scrollOffset > scrollLength) {
            scrollOffset = 0; // Reset scrolling to the start
        }
    }
}

void printFlash(){
  oled.invertDisplay(true);
}

void printInput(){
  // Check if there is incoming serial data
  if (Serial.available() > 0) {
    // Read the incoming message
    String inputMessage = Serial.readString();

    // Display the message on the OLED
    printFit(inputMessage.c_str());
  }

}