World world;
float dt = 0.016;

void setup() {
  size(800, 600, P3D);

  world = new World();

  for (int i = 0; i < 10; i++) {
    Vec3 p  = new Vec3(0, i * 20, 0);
    Quat q  = new Quat();
    Vec3 v0 = new Vec3(0, -0.1, 0);
    world.rod.addSegment(i, p, q, v0);
  }

  // On attache le premier segment
  world.rod.pinSegment(0);

  println("Base OK");
}

void draw() {
  background(0);

  lights();
  translate(width/2, height/4, 0);

  // Simulation
  world.update(dt);

  // Affichage
  world.drawRod();
  world.drawPoints();
}
