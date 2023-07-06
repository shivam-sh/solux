import processing.serial.*;

Serial serial;

int[] values = new int[5]; // Array to store the received values
float azimuth, elevation; // Variables for azimuth and elevation values

void setup() {
  fullScreen();
  serial = new Serial(this, Serial.list()[1], 9600); // Change the index [0] if needed
}

void draw() {
  background(0);
  
  if (serial.available() > 0) {
    String data = serial.readStringUntil('\n');
    if (data != null) {
      data = data.trim();
      String[] tokens = data.split("\t\t");
      
      if (tokens.length == 7) {
        for (int i = 0; i < 5; i++) {
          values[i] = Integer.parseInt(tokens[i].trim());
        }
        azimuth = Float.parseFloat(tokens[5].trim().substring(3));
        elevation = Float.parseFloat(tokens[6].trim().substring(3));
      }
    }
  }
  
  drawBarGraph();
  drawAzimuthElevation();
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

void drawAzimuthElevation() {
  float azX = map(azimuth, 0, 360, 0, width);
  float elY = map(elevation, 0, 90, height, 0);
  
  fill(225, 225, 200);
  ellipse(azX, elY, 10, 10);
  
  fill(255, 255, 200);
  textAlign(LEFT);
  textSize(20);
  text("AZ: " + azimuth, 20, 40);
  text("EL: " + elevation, 20, 80);
}
