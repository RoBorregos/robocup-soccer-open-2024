#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

void setup() {
  Serial.begin(115200); 
  Serial1.begin(115200); 
  Serial.println("Serial Passthrough");
}

void loop() {
  /*if (Serial1.available()) {
    String input1 =  Serial1.readStringUntil('\n');
    //Serial.println(input1);

    if (input1[0] == 'ballDistance'){
      Serial.println(input1[0]);
    }
  }*/
  /*if (Serial1.available()) { // Verifica si hay datos disponibles en UART1
    Serial.write(Serial1.read()); // Lee los datos de UART1 y los envía a UART0
  }*/
    if (Serial1.available() > 0) { 
 
    String data = Serial1.readStringUntil('\n');
    
   
    int values[4];
    int index = 0;
    char* ptr = strtok(const_cast<char*>(data.c_str()), ",");
    while (ptr != NULL && index < 4) {
      values[index++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    int distance_ball = values[0];
    int angle_ball = values[1];
    int angle_goal = values[2];
    int distance_goal = values[3];

    Serial.print("Distance Ball: ");
    Serial.println(distance_ball);
    Serial.print("Angle Ball: ");
    Serial.println(angle_ball);
    Serial.print("Angle Goal: ");
    Serial.println(angle_goal);
    Serial.print("Distance Goal: ");
    Serial.println(distance_goal);
  }
  
}