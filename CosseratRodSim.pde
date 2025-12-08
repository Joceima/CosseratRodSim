World world;

int grabbedId = -1;
float lastX, lastY;
boolean firstMousePress = false;


// ========== PARAMETERS ===========
// Segments parameters
float K_SS = 0.05f; // 0.03f 
float K_BT = 10.0f;
float STIFFNESS_FACTOR = 0.001f;
float MAX_CORRECTION = 0.9f;
float FACTOR_SS = 1f;
float GAMMA = 0.25f;
Vec3 gravity = new Vec3(0, 30, 0);

// Simulation parameters
float DT = 0.016;
int NB_OF_POINTS = 20;
int SUBSTEPS = 4;

float L0 = 10;

float EPS_3 = 0.001;
float EPS_6 = 0.000001;
float ESP_8 = 0.00000001;

HScrollbar[] scrollbars = new HScrollbar[5];
float[] values = {K_SS, K_BT, gravity.y, DT, NB_OF_POINTS};
String[] names = {"K_SS", "K_BT", "Gravité", "DT", "Points"};
float[] mins = {0.001, 0.1, 0, 0.001, 5};
float[] maxs = {1.0, 100.0, 5000.0, 0.05, 50};


void setup() {
  size(1000, 700, P3D);
  world = new World();
  
  createScrollbars();

  createRod();
}

void createRod() {
  world.rod.segments.clear();
  for (int i = 0; i < NB_OF_POINTS; i++) {
    Vec3 p  = new Vec3(0, i * 20, 0);
    Quat q  = new Quat();
    Vec3 v0 = new Vec3(0, 0, 0);
    world.rod.addSegment(i, p, q, v0, L0);
  }
  world.rod.pinSegment(0);
}

void createScrollbars() {
  int scrollbarWidth = 200;
  int scrollbarHeight = 16;
  int startX = 20;
  int startY = 40;
  int spacing = 45;
  int loose = 16;
  
  for (int i = 0; i < scrollbars.length; i++) {
    scrollbars[i] = new HScrollbar(
      startX, 
      startY + i * spacing, 
      scrollbarWidth, 
      scrollbarHeight, 
      loose
    );
    
    float initialPos = map(values[i], mins[i], maxs[i], 0, 1);
    scrollbars[i].setPos(initialPos);
  }
}

void draw() {
  background(0);
  lights();
  
  fill(30, 30, 40, 200);
  noStroke();
  rect(10, 10, 320, 310, 10);
  
  fill(255, 220, 100);
  textSize(16);
  textAlign(LEFT, TOP);
  text("CONTROLE DES PARAMÈTRES", 20, 20);
  
  for (int i = 0; i < scrollbars.length; i++) {
    scrollbars[i].update();
    scrollbars[i].display();
    
    float pos = scrollbars[i].getPos();
    values[i] = map(pos, 0, 1, mins[i], maxs[i]);
    if (names[i].equals("Points")) {
      values[i] = int(values[i]);
    }
    
    fill(180, 220, 255);
    textSize(12);
    textAlign(LEFT, CENTER);
    
    String displayValue;
    if (names[i].equals("Points")) {
      displayValue = str(int(values[i]));
    } else if (values[i] >= 1000) {
      displayValue = nf(values[i], 0, 0);
    } else if (values[i] >= 100) {
      displayValue = nf(values[i], 0, 0);
    } else if (values[i] >= 10) {
      displayValue = nf(values[i], 0, 1);
    } else {
      displayValue = nf(values[i], 0, 3);
    }
    
    text(names[i] + ": " + displayValue, 
         240, scrollbars[i].ypos + scrollbars[i].sheight/2);
  }
  
  // Mettre à jour les paramètres de la simulation
  updateSimulationParameters();
  
  pushMatrix();
  translate(width/2, height/4, 0);
  rotateX(0.8);
  
  // Simulation physique
  world.update();
  
  // Affichage
  world.drawRod();
  world.drawPoints();
  popMatrix();
  
  // Après utilisation, réinitialiser firstMousePress
  if (firstMousePress) {
    firstMousePress = false;
  }
}

void updateSimulationParameters() {
  K_SS = values[0];
  K_BT = values[1];
  gravity.y = values[2];
  DT = values[3];
  
  for (Segment s : world.rod.segments) {
    s.k_ss = K_SS;
    s.k_bt = K_BT;
  }
  
  if (int(values[4]) != NB_OF_POINTS) {
    NB_OF_POINTS = int(values[4]);
    createRod();
  }
}

void mousePressed() {
  if (!firstMousePress) {
    firstMousePress = true;
  }
  
  grabbedId = pickSegment();
}

void mouseDragged() {
  if (grabbedId == -1) return;
  Segment s = world.rod.segments.get(grabbedId);
  if(s.pinned) return;

  Vec3 target = mouseToWorld();
  s.p = target.copy();
  s.p_pred = target.copy();
}

void mouseReleased() {
  grabbedId = -1;
}

// ========================
Vec3 mouseToWorld() {
  float X = mouseX - width/2;
  float Y = mouseY - height/4;

  float cosr = cos(0.8);
  float sinr = sin(0.8);

  float wy = Y*cosr;
  float wz = -Y*sinr;

  return new Vec3(X, wy, wz);
}

int pickSegment() {
  pushMatrix();                      
  translate(width/2, height/4, 0);  
  rotateX(0.8);                      

  int nearestId = -1;
  float minDist = 10;   

  for (int i = 0; i < world.rod.segments.size(); i++) {
    Segment s = world.rod.segments.get(i);

    float sx = screenX(s.p.x, s.p.y, s.p.z);
    float sy = screenY(s.p.x, s.p.y, s.p.z);

    float d = dist(mouseX, mouseY, sx, sy);

    if (d < minDist) {
      minDist = d;
      nearestId = i;
    }
  }

  popMatrix();
  return nearestId;
}
