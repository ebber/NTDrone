void setup()
{
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  //ofset for data coming in is 48; ie 48=0,49=1,50=2
  //XBee.begin(9600);
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  delay(4000);
  Serial.write("Hello there");
}

void loop()
{
  if (Serial.available()) {
    int val=Serial.read(); //get input
    if(val==13) { //if enter is hit
     Serial.print("Value is: ");
     Serial.print(getValue());
     Serial.println();

      counter=0;
      val=value[0]+offset; //dont change values
    
  } else {
      
      value[counter%(digitsTaking)]=val-offset; //48 is offset for ints
      
      if(counter==digitsTaking-1) {
        counter=0; //reset
     Serial.print("Value is: ");
     Serial.print(getValue());
     Serial.println();      } 
     else {
        counter=counter+1; //increment counter
      }
    }
  }
}
int getValue() {
   int v = 0;
   for(int i=0; i<digitsTaking; i++) {
        v=v+value[i]* (int) pow(10, digitsTaking-i-1);
        Serial.println(value[i]);
        Serial.println( (int) value[i]*pow(10, digitsTaking-i-1) );
   } 
  return v;
}
