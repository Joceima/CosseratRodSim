class World {
  Rod rod;

  World() {
    rod = new Rod();
  }

  void update(float dt) {
    // 1) Forces externes (gravité)
    rod.applyGravity(dt, new Vec3(0, 200, 0));

    // 2) Intégration explicite
    rod.integrateExplicit(dt);

    // 3) Contraintes (distance)
    // itérer plusieurs fois améliore la rigidité
    int iters = 4;
    for (int k = 0; k < iters; k++) rod.projectConstraints();

    // 4) Orientation simple (lookAt)
    rod.updateOrientationSimple();

    // 5) (optionnel) Stretch/shear forces — désactivés si tu veux
    rod.computeStretchShearForces();
    rod.applyStretchShear(dt);

    // 6) (optionnel) Bending/twisting
    rod.computeBendingTwistingTorques();
    rod.applyBendingTwisting(dt);

    // 7) Damping final
    rod.applyDamping(0.98f);
  }

  void drawPoints() {
    noStroke();
    fill(255, 180, 0);
    for (Segment s : rod.segments) {
      pushMatrix();
      translate(s.p.x, s.p.y, s.p.z);
      sphere(4);
      popMatrix();
    }
  }

  void drawRod() {
    stroke(255);
    strokeWeight(3);
    for (int i = 0; i < rod.segments.size() - 1; i++) {
      Segment a = rod.segments.get(i);
      Segment b = rod.segments.get(i + 1);
      line(a.p.x, a.p.y, a.p.z, b.p.x, b.p.y, b.p.z);
    }
  }
}
