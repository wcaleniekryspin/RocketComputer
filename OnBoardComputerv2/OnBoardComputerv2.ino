#include "Rakieta.h"


Rakieta rakieta;

void setup()
{
  debugInit(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) delay(10);

  rakieta.init();
}

void loop()
{
  rakieta.loop();
}
