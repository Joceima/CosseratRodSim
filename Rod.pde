class Rod {
  ArrayList<Segment> segments = new ArrayList<Segment>();

  void addSegment(int idx, Vec3 p, Quat q, Vec3 v0, float l0) {
    Segment seg = new Segment(idx, p, q, v0, l0);
    seg.Q_rest = q.copy();
    segments.add(seg);
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

  void updateOrientationSimple() {
    for (int i = 0; i < segments.size() - 1; i++) {
      Segment a = segments.get(i);
      Segment b = segments.get(i+1);
      Vec3 dir = b.p.sub(a.p);
      a.q = a.q.lookAt(dir); 
    }
    if (segments.size() > 1) {
      segments.get(segments.size()-1).q = segments.get(segments.size()-2).q.copy();
    }
  }

  void step(float h) {

    // === ÉTAPE 1: Prédiction des positions ===
    Vec3 gravity = new Vec3(0, 2000, 0);
    for (Segment s : segments) {
      if (!s.pinned) {
        s.p_pred = s.p.add(s.v.mul(h));
      } else {
        s.p_pred = s.p.copy();
        s.v = new Vec3(0, 0, 0);
      }
    }

    // Ajoutez gravité comme une simple force
    for (Segment s : segments) {
      if (!s.pinned) {
        s.p_pred.y += 0.5f * gravity.y * h * h; // Formule de chute libre: y = ½gt²
      }
    }
    // === ÉTAPE 2: Résolution des contraintes sur p_pred ===
    for (int iter = 0; iter < 10; iter++) {
      for (int i = 0; i < segments.size() - 1; i++) {
        solveDistanceConstraintOnPredicted(i);
      }
    }


    applyCosseratStretching(h);

    // === ÉTAPE 3: Mise à jour des vitesses et positions ===
    for (Segment s : segments) {
      if (!s.pinned) {
        s.v = s.p_pred.sub(s.p).mul(1.0f / h);
        s.v = s.v.mul(0.98f);
        s.p = s.p_pred.copy();
      }
    }
    updateOrientationSimple();
  }


  void solveDistanceConstraintOnPredicted(int i) {
    Segment a = segments.get(i);
    Segment b = segments.get(i+1);
    
    if (a.pinned && b.pinned) return;
    
    Vec3 delta = b.p_pred.sub(a.p_pred);
    float d = delta.length();
    //println("delta " + d);
    if (d < 1e-6) return;
    
    float rest = a.l0;
    float diff = (d - rest) / d;
    
    float resistance = a.k_ss * 0.1f;  // Facteur d'échelle
    float correctionFactor = diff * resistance;
    correctionFactor = constrain(correctionFactor, -0.5f, 0.5f);
  
    Vec3 corr = delta.mul(0.5f * correctionFactor);
    
    if (!a.pinned) a.p_pred = a.p_pred.add(corr);
    if (!b.pinned) b.p_pred = b.p_pred.sub(corr);
  }


  void applyCosseratStretching(float h) {
    // v = −2·k_ss·(1/l0)·(x_{i+1} - x_i)
    for (int i = 0; i < segments.size() - 1; i++) {
      Segment a = segments.get(i);
      Segment b = segments.get(i+1);

      Vec3 dx = b.p_pred.sub(a.p_pred);
      float l0 = a.l0;

      Vec3 stretchingForce = dx.mul(-2.0f * a.k_ss / l0);

      if (!a.pinned) a.p_pred = a.p_pred.add(stretchingForce.mul(0.1f * h));
      if (!b.pinned) b.p_pred = b.p_pred.sub(stretchingForce.mul(0.1f * h));
    }
  }
}
