import processing.serial.*;

Serial serial;

int[] values = new int[5]; // Array to store the received values

void setup() {
fullScreen();  serial = new Serial(this, Serial.list()[1], 9600); // Change the index [0] if needed
}

void draw() {
  background(0);
  
  if (serial.available() > 0) {
    String data = serial.readStringUntil('\n');
    if (data != null) {
      data = data.trim();
      String[] tokens = data.split("\t\t");
      
      if (tokens.length == 5) {
        for (int i = 0; i < tokens.length; i++) {
          values[i] = Integer.parseInt(tokens[i]);
        }
      }
    }
  }
  
  drawBarGraph();
}

void drawBarGraph() {
  float barWidth = width / values.length;
  
  for (int i = 0; i < values.length; i++) {
    float barHeight = map(values[i], 0, 4095, 0, height);
    float x = i * barWidth;
    float y = height - barHeight;
    
    fill(255, 255, 200);
    rect(x, y, barWidth, barHeight);
    
    fill(0);
    textAlign(CENTER);
    text(values[i], x + barWidth / 2, y - 5);
  }
}
