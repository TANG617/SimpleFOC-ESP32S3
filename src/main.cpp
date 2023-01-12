//TEST FOR SERIAL PRINT
#include<Arduino.h>
void setup()
{
  Serial0.begin(115200);
}

void loop()
{
  Serial0.println("TEST,OK");
  delay(1000);
}