#include "Rakieta.h"

Rakieta rakieta;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  rakieta.init();
}

void loop()
{
  rakieta.loop();
}
