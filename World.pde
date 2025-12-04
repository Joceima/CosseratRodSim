class World {
  Rod rod;

  World() {
    rod = new Rod();
  }

  void update(float dt) {
    rod.applyGravity(dt, new Vec3(0, 200, 0)); 
    rod.integrateExplicit(dt);
    rod.applyDamping(0.98);
  }

  void drawPoints() {
    for (Segment s : rod.segments) {
      stroke(255, 200, 0);
      fill(255, 180, 0);
      pushMatrix();
      translate(s.p.x, s.p.y, s.p.z);
      sphere(5);
      popMatrix();
    }
  }

  void drawRod() {
    stroke(255);
    strokeWeight(4);

    for (int i = 0; i < rod.segments.size() - 1; i++) {
      Segment a = rod.segments.get(i);
      Segment b = rod.segments.get(i + 1);
      line(a.p.x, a.p.y, a.p.z, b.p.x, b.p.y, b.p.z);
    }
  }

}
