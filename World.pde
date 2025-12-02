class World {
  Rod rod;

  World() {
    rod = new Rod();
  }

  void update(float dt) {
    // vide pour l’instant
  }

  void draw() {
    for (Segment s : rod.segments) {
      stroke(255, 200, 0);
      fill(255, 180, 0);
      pushMatrix();
      translate(s.p.x, s.p.y, s.p.z);
      sphere(5);
      popMatrix();
    }
  }
}
