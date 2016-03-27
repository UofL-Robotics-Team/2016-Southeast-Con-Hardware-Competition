void setup() 
{
  pinMode(52, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
}

void loop()
{
  digitalWrite(52, LOW);
  digitalWrite(8, LOW);

  for(int i = 0; i < 6000; i++)
  {
    digitalWrite(9, HIGH);
    delay(1);
    digitalWrite(9, LOW);
    delay(1);
  }

  digitalWrite(8, HIGH);

  for(int i = 0; i < 6000; i++)
  {
    digitalWrite(9, HIGH);
    delay(1);
    digitalWrite(9, LOW);
    delay(1);
  }
}
