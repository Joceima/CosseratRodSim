World world;

void setup() {
  size(800, 600, P3D);

  world = new World();

  // On ajoute quelques segments juste pour tester
  for (int i = 0; i < 5; i++) {
    world.rod.addSegment(i, new Vec3(i * 20, 0, 0), new Quat());
  }

  println("Base OK");
}

void draw() {
  background(0);
  lights();
  translate(width/2, height/2, 0);

  world.update(1.0/60.0);
  world.draw();
}
