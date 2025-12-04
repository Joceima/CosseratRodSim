World world;
float dt = 0.016;

int grabbedId = -1;
float lastX, lastY;

void setup() {
  size(900, 700, P3D);
  world = new World();

  // --- création d'une corde simple ---
  for (int i = 0; i < 12; i++) {
    Vec3 p  = new Vec3(0, i * 20, 0);
    Quat q  = new Quat();
    Vec3 v0 = new Vec3(0, 0, 0);
    float l0 = 20;

    world.rod.addSegment(i, p, q, v0, l0);
  }

  world.rod.pinSegment(0);

  println("Simulation prête !");
}

void mousePressed() {
  lastX = mouseX;
  lastY = mouseY;
  grabbedId = pickSegment();
  println("grabbedId = " + grabbedId);
}

void mouseDragged() {
  if (grabbedId != -1) {
    Segment s = world.rod.segments.get(grabbedId);

    pushMatrix();
    translate(width/2, height/4, 0);
    rotateX(0.8);

    float sx = screenX(s.p.x, s.p.y, s.p.z);
    float sy = screenY(s.p.x, s.p.y, s.p.z);

    popMatrix();

    float dx = mouseX - sx;
    float dy = mouseY - sy;

    // ---------------------------
    Vec3 F = new Vec3(dx * 3.0f, dy * -3.0f, 0);


    world.rod.applyForceTo(grabbedId, F);
  }
}


void mouseReleased() {
  grabbedId = -1;
}

int pickSegment() {

  pushMatrix();                      
  translate(width/2, height/4, 0);  
  rotateX(0.8);                      

  int nearestId = -1;
  float minDist = 25;   // capture un peu plus large

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


PVector worldToScreen(float x, float y, float z) {
  // simple wrapper to Processing helpers
  return new PVector(screenX(x, y, z), screenY(x, y, z));
}

void draw() {
  background(0);
  lights();

  // transformation graphique (tu peux changer)
  pushMatrix();
  translate(width/2, height/4, 0);
  rotateX(0.8);

  // --- physique ---
  world.update(dt);

  // --- affichage ---
  world.drawRod();
  world.drawPoints();

  popMatrix();
}
