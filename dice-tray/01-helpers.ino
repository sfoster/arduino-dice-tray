String hsvToString(CHSV color) {
  return "{h:" + String(color.h) + ",s:" + String(color.s) + ",v:" + String(color.v) + "}";
}

int getProgress(int i, int upper, int total) {
  float propn = (float)i/(float)upper;
  return (int)(total * propn) % total; 
}

void updateAllRows(CRGB* pixels, PixelRow** rows, int rowCount, CHSV color) {
  for (int rowIdx=0; rowIdx<rowCount; rowIdx++) {
    updateRow(pixels, rows[rowIdx], color);
  }
}

void updateRow(CRGB* pixels, PixelRow* rowPtr, CHSV color) {
  for (int col=0; col<rowPtr->len; col++) {
    pixels[rowPtr->pixelOffset + col] = color;
  }
}

void fadeRow(PixelRow* rowPtr, byte decrement) {
  for (int col=0; col<rowPtr->len; col++) {
    leds[rowPtr->pixelOffset + col].subtractFromRGB(decrement);
  }
}

void rainbowClimb(int progress) {
  byte hueOffset = 0;
  for (int i=0; i<ROW_COUNT; i++) {
    hueOffset = 30 * i;
    updateRow(leds, allRows[i], CHSV(hueOffset+progress, 255, 255));
  }
}
