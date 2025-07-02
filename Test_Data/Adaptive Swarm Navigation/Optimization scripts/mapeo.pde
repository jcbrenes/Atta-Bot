import processing.video.*;
import java.io.*;

// Variable for capture device
Capture video;

// Variables for the colors we are searching for.
color trackColor1;
color trackColor2;
color trackColor3;
color blanco = color(255,255,255);

// Constants for square size and number of squares
final int SQUARE_SIZE = 20;
final int RECT_WIDTH = 22;
final int NUM_SQUARES_X = 24;
final int NUM_SQUARES_Y = 24;

// Variables for divided color matrix
float[][] dividedColorMatrix;

float threshold = 30;

boolean[][] colorMatrix; // Boolean matrix to store pixel information
boolean[][] reducedColorMatrix; // Boolean matrix for the reduced matrix

boolean primerClic = true;
boolean primerClic2 = true;

int lastSaveTime = 0;
int saveInterval = 10000; // Save interval in milliseconds (10 seconds)

void setup() {
  background(1);
  size(1280, 480);
  String[] cameras = Capture.list();
  printArray(cameras);
  video = new Capture(this, cameras[1]);
  video.start();
  trackColor1 = color(255, 0, 0);
  trackColor2 = color(0, 0, 255);
  trackColor3 = color(0, 255, 0);
  colorMatrix = new boolean[video.height][video.width]; // Initialize color matrix
  reducedColorMatrix = new boolean[video.height][video.width - 112]; // Initialize reduced matrix
  dividedColorMatrix = new float[NUM_SQUARES_Y][NUM_SQUARES_X];
}

void captureEvent(Capture video) {
  // Read image from the camera
  video.read();
}

void draw() {
  video.loadPixels();
  image(video, 0, 0);

  // Begin loop to walk through every pixel
  for (int x = 0; x < video.width; x++) {
    for (int y = 0; y < video.height; y++) {
      int loc = x + y * video.width;
      // What is current color
      color currentColor = video.pixels[loc];
      float r1 = red(currentColor);
      float g1 = green(currentColor);
      float b1 = blue(currentColor);

      float r2 = red(trackColor1);
      float g2 = green(trackColor1);
      float b2 = blue(trackColor1);

      float d1 = dist(r1, g1, b1, r2, g2, b2); // We are using the dist( ) function to compare the current color with the color we are tracking.
      
      float r3 = red(trackColor2);
      float g3 = green(trackColor2);
      float b3 = blue(trackColor2);

      float d2 = dist(r1, g1, b1, r3, g3, b3); // We are using the dist( ) function to compare the current color with the color we are tracking.
      
      float r4 = red(trackColor3);
      float g4 = green(trackColor3);
      float b4 = blue(trackColor3);

      float d3 = dist(r1, g1, b1, r4, g4, b4); // We are using the dist( ) function to compare the current color with the color we are tracking.
      
      // If current color is similar to tracked color, draw a pixel on the right frame and mark the position as checked
      if (d1 < threshold || d2 < threshold || d3 < threshold) {
        fill(blanco);
        strokeWeight(0);
        stroke(1.0);
        ellipse(x + video.width, y, 1, 1); 
        colorMatrix[y][x] = true; // Set the corresponding pixel in the matrix to true
      }
    }
  }
  
  // Populate reducedColorMatrix
  for (int y = 0; y < video.height; y++) {
    for (int x = 16; x < video.width - 96; x++) {
      reducedColorMatrix[y][x - 16] = colorMatrix[y][x];
    }
  }

  // Iterate over each cell of the divided matrix
  for (int filaDividida = 0; filaDividida < NUM_SQUARES_Y; filaDividida++) {
    for (int columnaDividida = 0; columnaDividida < NUM_SQUARES_X; columnaDividida++) {  
      float promedioUnos = 0;
      int sumatoriaUnos = 0;
      // Iterate over each pixel within the cell
      for (int filaPixeles = filaDividida * SQUARE_SIZE; filaPixeles < (filaDividida + 1) * SQUARE_SIZE; filaPixeles++) {
        for (int columnaPixeles = columnaDividida * RECT_WIDTH; columnaPixeles < (columnaDividida + 1) * RECT_WIDTH; columnaPixeles++) {
          if (filaPixeles < video.height && columnaPixeles < video.width - 112 && reducedColorMatrix[filaPixeles][columnaPixeles]) {
            sumatoriaUnos++;
          }
        }
      }
      promedioUnos = (float) sumatoriaUnos / (SQUARE_SIZE * RECT_WIDTH);
      dividedColorMatrix[filaDividida][columnaDividida] = promedioUnos;
    }
  }
  
  // Check if it's time to save matrices
  if (millis() - lastSaveTime >= saveInterval) {
    // Save matrices with different names
    String timestamp = nf(hour(), 2) + "-" + nf(minute(), 2) + "-" + nf(second(), 2);
    guardarMatrizPixeles(reducedColorMatrix, "prueba/reduced_color_matrix/reduced_color_matrix_" + timestamp + ".csv");
    guardarMatrizDividida(dividedColorMatrix, "prueba/divided_color_matrix/divided_color_matrix_" + timestamp + ".csv");
    lastSaveTime = millis(); // Reset last save time
  }
}

void mousePressed() {
  // Toggle between three colors on mouse press
  int loc = mouseX + mouseY * video.width;
  if (primerClic) {
    trackColor1 = video.pixels[loc];
    primerClic = false;
  } else if (primerClic2) {
    trackColor2 = video.pixels[loc];
    primerClic2 = false;
  } else {
    trackColor3 = video.pixels[loc];
    primerClic = true;
    primerClic2 = true;
  }
}

void keyPressed() {
  if (key == 's' || key == 'S') { // Press 's' or 'S' to save the matrix
    guardarMatrizPixeles(reducedColorMatrix, "color_matrix.csv");
    println("Color matrix saved as color_matrix.csv");
  }
}

void guardarMatrizDividida(float[][] matriz, String nombreArchivo) {
  PrintWriter archivo = createWriter(nombreArchivo);
  for (int i = 0; i < matriz.length; i++) {
    for (int j = 0; j < matriz[0].length; j++) {
      archivo.print(matriz[i][j]);
      if (j < matriz[0].length - 1) {
        archivo.print(","); // Use comma as delimiter
      }
    }
    archivo.println();
  }
  archivo.flush();
  archivo.close();
}

void guardarMatrizPixeles(boolean[][] matriz, String nombreArchivo) {
  PrintWriter archivo = createWriter(nombreArchivo);
  for (int i = 0; i < matriz.length; i++) {
    for (int j = 0; j < matriz[0].length; j++) {
      archivo.print(matriz[i][j] ? "1" : "0");
      if (j < matriz[0].length - 1) {
        archivo.print(","); // Use comma as delimiter
      }
    }
    archivo.println();
  }
  archivo.flush();
  archivo.close();
}
