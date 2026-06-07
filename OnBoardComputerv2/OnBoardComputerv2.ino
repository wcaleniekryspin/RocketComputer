#include "Rakieta.h"


Rakieta rakieta;

void setup()
{
  debugInit(115200);

  Wire.begin();
  rakieta.init();
}

void loop()
{
  rakieta.loop();
}
