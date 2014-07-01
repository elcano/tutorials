/* Serial pass through routine for C4
   Tyler Folsom 4/10/14
   
   Part of circle serial test: C6-> C4 -> C3 -> C6
   */
   
void setup() 
{ 
    Serial.begin(115200); // C6 to C4 to C3
}
void loop()
{
    int incomingByte = 0;   // for incoming serial data
    // send data only when you receive data:
        while (Serial.available() > 0) 
        {
                // read the incoming byte from C6:
                incomingByte = Serial.read();
                // pass it on to C3
                Serial.write(incomingByte);
        }
}
