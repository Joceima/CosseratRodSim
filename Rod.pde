final float EPS_3 = 0.001;
final float EPS_6 = 0.000001;

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
    applyCosseratBending(h);
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
    if (d < 1e-6) return;
    
    float rest = a.l0;
    float diff = (d - rest) / d;
    
    float resistance = a.k_ss * 0.1f;  
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

  void applyCosseratBending(float h)
  {
    // b = 
    for(int i = 1; i < segments.size() -1; i++)
    {
      Segment prev = segments.get(i-1);
      Segment current = segments.get(i);
      Segment next = segments.get(i+1);

      Quat term1 = prev.q.mul(prev.Q_rest.conjugate());

      term1 = term1.mul(prev.phi);
      term1 = term1.mul(prev.k_bt);

      Quat term2 = next.q.mul(current.Q_rest.conjugate());

      term2 = term2.mul(current.phi);
      term2 = term2.mul(current.k_bt);

      Quat b = term1.add(term2);

      Vec3 v = computeStretchingEffectAtPoint(i);

      float lambda = computeBendingLambda(i,v,b);

      updateQuaternionWithBending(i,v,b, lambda);
      updatePhiField(i,v,b, lambda,h);
    }
  }

  Vec3 computeStretchingEffectAtPoint(int i) {
   if (i <= 0 || i >= segments.size()-1) {
     return new Vec3(0, 0, 0);
   }

   Segment leftSeg = segments.get(i-1);
   Segment rightSeg = segments.get(i);

   //v_left = -2 * k_ss * (1/l0) * (x_i - x_{i-1})
   Vec3 dx_left = segments.get(i).p_pred.sub(segments.get(i-1).p_pred);
   Vec3 v_left = dx_left.mul(-2.0f * leftSeg.k_ss / leftSeg.l0);

   //  v_right = -2 * k_ss * (1/l0) * (x_{i+1} - x_i)
   Vec3 dx_right = segments.get(i+1).p_pred.sub(segments.get(i).p_pred);
   Vec3 v_right = dx_right.mul(-2.0f * rightSeg.k_ss / rightSeg.l0);

   return v_left.add(v_right).mul(0.5f);
  } 


  float computeBendingLambda(int i, Vec3 v, Quat b) {
    Segment s = segments.get(i);

    float vNorm = v.length();
    float bNorm = b.norm();

    if (bNorm < EPS_6) {
      s.gamma = 0.1f;
      return 0.1f;
    }
    float gammaClamped = constrain(s.gamma, EPS_3, 1.0f);
    float lambda = vNorm + gammaClamped * bNorm;
    float v_e3 = v.z;
    float inside = abs(v_e3 + lambda * bNorm) + vNorm * vNorm;
    lambda = sqrt(inside);

    s.gamma = (lambda - vNorm) / bNorm;

    lambda = constrain(lambda, 0.001f, 10.0f);
    s.gamma = constrain(s.gamma, 0.001f, 1.0f);

    return lambda;
  }

  void updateQuaternionWithBending(int i, Vec3 v, Quat b, float lambda) {
    Segment s = segments.get(i);
    
    // q_i = v·e3 + λ·b
    Vec3 e3 = new Vec3(0, 0, 1);
    
    Vec3 ve3 = new Vec3(v.x * e3.x, v.y * e3.y, v.z * e3.z);
    Quat q_ve3 = new Quat(0, ve3.x, ve3.y, ve3.z);
    
    // λ·b
    Quat q_lambda_b = b.mul(lambda);
    
    Quat q_new = q_ve3.add(q_lambda_b);
    q_new.normalize();
    
    float blend = 0.1f; 
    s.q = s.q.slerp(q_new, blend);
  }

  void updatePhiField(int i, Vec3 v, Quat b, float lambda, float h) {
    Segment s = segments.get(i);
    float alpha = 0.01f * h; 

    Vec3 e3 = new Vec3(0, 0, 1);
    Vec3 ve3 = new Vec3(v.x * e3.x, v.y * e3.y, v.z * e3.z);
    Quat q_ve3 = new Quat(0, ve3.x, ve3.y, ve3.z);
    Quat q_lambda_b = b.mul(lambda);
    Quat q_target = q_ve3.add(q_lambda_b);

    Quat q_diff = s.q.mul(q_target.conjugate());


    Quat phi_update = q_diff.mul(alpha);
    s.phi = s.phi.sub(phi_update);

    float maxPhi = 0.1f;
    if (s.phi.norm() > maxPhi) {
      s.phi = s.phi.normalized().mul(maxPhi);
    }
  }

}
