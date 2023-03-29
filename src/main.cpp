//TEST FOR SERIAL PRINT
#include"func.h"
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.println("TEST,OK");
  delay(1000);
}