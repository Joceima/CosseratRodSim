class Rod {
  ArrayList<Segment> segments = new ArrayList<Segment>();

  void addSegment(int idx, Vec3 p, Quat q, Vec3 v0, float l0) {
    segments.add(new Segment(idx, p, q, v0, l0));
  }

  void pinSegment(int id) {
    if (id >= 0 && id < segments.size()) {
      segments.get(id).pinned = true;
      segments.get(id).v = new Vec3(0,0,0);
    }
  }

  void applyGravity(float dt, Vec3 g) {
    for (Segment s : segments) {
      if (!s.pinned) s.v = s.v.add(g.mul(dt));
    }
  }

  void applyForceTo(int id, Vec3 F) {
    if (id < 0 || id >= segments.size()) return;
    Segment s = segments.get(id);
    if (s.pinned) return;
    // scale down impulse so drag isn't huge
    s.v = s.v.add(F.mul(0.05f));
  }

  void integrateExplicit(float dt) {
    for (Segment s : segments) {
      if (!s.pinned) s.p = s.p.add(s.v.mul(dt));
    }
  }

  void applyDamping(float factor) {
    for (Segment s : segments) s.v = s.v.mul(factor);
  }

  // distance constraint between i and i+1
  void solveDistanceConstraint(int i) {
    Segment a = segments.get(i);
    Segment b = segments.get(i+1);
    Vec3 delta = b.p.sub(a.p);
    float d = delta.length();
    if (d < 1e-6) return;
    float rest = a.l0;
    float diff = (d - rest) / d;
    Vec3 corr = delta.mul(0.5f * diff);
    if (!a.pinned) a.p = a.p.add(corr);
    if (!b.pinned) b.p = b.p.sub(corr);
  }

  void projectConstraints() {
    for (int i = 0; i < segments.size()-1; i++) {
      solveDistanceConstraint(i);
    }
  }

  // orientation simple: make quaternion face next segment
  void updateOrientationSimple() {
    for (int i = 0; i < segments.size() - 1; i++) {
      Segment a = segments.get(i);
      Segment b = segments.get(i+1);
      Vec3 dir = b.p.sub(a.p);
      a.q = a.q.lookAt(dir); // instance method that returns a Quat
    }
    // last copy from previous
    if (segments.size() > 1) {
      segments.get(segments.size()-1).q = segments.get(segments.size()-2).q.copy();
    }
  }

  // --- minimal (safe) implementations of Cosserat steps ---
  void computeStretchShearForces() {
    // keep minimal: store nothing (optionally compute forces)
    for (int i = 0; i < segments.size()-1; i++) {
      Segment a = segments.get(i);
      Segment b = segments.get(i+1);
      // you can compute forces here. For now we do nothing so system remains stable.
    }
  }
  void applyStretchShear(float dt) { /* no-op for now */ }

  void computeBendingTwistingTorques() {
    // minimal no-op (safe)
    for (Segment s : segments) {
      // ensure fields exist
      s.tau = new Vec3(0,0,0);
    }
  }
  void applyBendingTwisting(float dt) { /* no-op */ }
}
