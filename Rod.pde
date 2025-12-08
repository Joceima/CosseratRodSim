class Rod {
  ArrayList<Segment> segments = new ArrayList<Segment>();

  static final float FACTOR_SS = 0.01f;
  static final float STIFFNESS_FACTOR = 0.001f;
  static final float MAX_CORRECTION = 0.9f;
  static final int N_ITERATIONS = 5;
  static final float MAX_ORIENTATION_CORRECTION = 0.9f;

  void initializePhiFields() {
    for (Segment seg : segments) {
      seg.phi = new Quat(1, 0, 0, 0);
      seg.Q_rest = seg.q.copy();
      seg.gamma = 0.5f;
    }
  }

  void addSegment(int idx, Vec3 p, Quat q, Vec3 v0, float l0) {
    Segment seg = new Segment(idx, p, q, v0, l0);
    seg.Q_rest = q.copy();
    seg.phi = new Quat(1, 0, 0, 0);
    seg.gamma = 0.5f;
    segments.add(seg);
  }

  void pinSegment(int id) {
    if (id >= 0 && id < segments.size()) {
      segments.get(id).pinned = true;
      segments.get(id).v = new Vec3(0, 0, 0);
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

  void applyGravity(float h, Vec3 gravity) {
    for (Segment s : segments) {
      if (!s.pinned) {
        s.p_pred = s.p.add(s.v.mul(h)).add(gravity.mul(h * h * 0.5f));
      } else {
        s.p_pred = s.p.copy();
        s.v = new Vec3(0, 0, 0);
      }
    }
  }

  // deepseek
  // VBD simple pour positions
  void solveDistanceConstraintOnPredictedVBD(int i) {
    Segment a = segments.get(i);
    Segment b = segments.get(i+1);
    
    if (a.pinned && b.pinned) return;
    
    Vec3 delta = b.p_pred.sub(a.p_pred);
    float d = delta.length();
    if (d < EPS_6) return;
    
    float rest = a.l0;
    
    // PERMET LE STRETCHING selon k_ss
    // Plus k_ss est grand, moins on permet d'étirement
    float targetLength = rest;
    if (d > rest) {
      // Étirement permis selon la compliance (inverse de la raideur)
      float stretchRatio = (d - rest) / rest;
      float compliance = 1.0f / (1.0f + a.k_ss * 0.1f);
      targetLength = rest * (1.0f + stretchRatio * compliance);
    }
    // Corrige seulement la compression excessive
    else if (d < rest * 0.8f) {
      targetLength = rest * 0.8f;
    }
    
    float diff = (d - targetLength) / d;
    float stiffness = 0.5f;
    //float stiffness = 0.1f * a.k_ss;
    //stiffness = constrain(stiffness, 0.01f, 0.5f);
    
    Vec3 corr = delta.mul(0.5f * diff * stiffness);
    
    if (!a.pinned) a.p_pred = a.p_pred.add(corr);
    if (!b.pinned) b.p_pred = b.p_pred.sub(corr);
  }

  void step(float h) {
    // === ÉTAPE 1: Initialisation ===
    applyGravity(h, gravity);
    
    // === ÉTAPE 2: Résolution itérative ===
    for (int iter = 0; iter < N_ITERATIONS; iter++) {
      
      // --- PARTIE A: Mise à jour des positions (VBD) ---
      for (int i = 0; i < segments.size() - 1; i++) {
        solveDistanceConstraintOnPredictedVBD(i);
      }
      
      // --- PARTIE B: Mise à jour des orientations ---
      for (int i = 1; i < segments.size() - 1; i++) {
        // 1. Calcul de v (Équation 15)
        Vec3 v = computeStretchingEffectAtPoint(i);
        
        // 2. Calcul de b (Équation 17)
        Quat b = computeBendingEffectAtPoint(i);
        
        // 3. Calcul de λ
        float lambda = iterateLambda(i, v, b);
        
        // 4. Mise à jour de l'orientation (Équation 18)
        updateQuaternionWithStretchingBending(i, v, b, lambda);
        //updateQuaternionSimple(i,v,b,lambda);
        // 5. Normalisation
        segments.get(i).q.normalize();
        
        // 6. Mettre à jour phi 
        updatePhiField(i, h);
      }
    }
    
    // === ÉTAPE 3: Mise à jour finale ===
    for (Segment s : segments) {
      if (!s.pinned) {
        s.v = s.p_pred.sub(s.p).mul(1.0f / h);
        s.v = s.v.mul(0.80f); // Damping
        s.p = s.p_pred.copy();
      }
    }
  }

  float computePhiValue(int edgeIndex) {
    if (edgeIndex < 0 || edgeIndex >= segments.size() - 1) return 1.0f;
    
    Segment left = segments.get(edgeIndex);
    Segment right = segments.get(edgeIndex + 1);
    
    // q_diff = qq_i * q_{i+1}  (différence de rotation entre segments)
    Quat q_diff = right.q.mul(left.q.conjugate());
    
    Quat diff_minus = q_diff.sub(right.Q_rest);
    Quat diff_plus = q_diff.add(right.Q_rest);
    
    float dist_minus = diff_minus.norm() * diff_minus.norm(); // norm()²
    float dist_plus = diff_plus.norm() * diff_plus.norm();    // norm()²
    
    return (dist_minus <= dist_plus) ? 1.0f : -1.0f;
  }

  float iterateLambda(int i, Vec3 v, Quat b) {
    if (i < 0 || i >= segments.size()) {
      return 0.0f;
    }
    Segment seg = segments.get(i);
    float v_norm = v.length();
    float b_norm = b.norm();
    float gamma_clamped = constrain(seg.gamma, 1e-3f, 1.0f);
    float lambda = v_norm + gamma_clamped * b_norm;
    
    if (b_norm > 1e-6f) {
      
      Vec3 e3 = new Vec3(0, 0, 1);
      Vec3 v_dot_e3 = new Vec3(v.x * e3.x, v.y * e3.y, v.z * e3.z);
      Quat q_v_dot_e3 = new Quat(0, v_dot_e3.x, v_dot_e3.y, v_dot_e3.z);
      Quat q_lambda_b = b.mul(lambda);
      Quat sum = q_v_dot_e3.add(q_lambda_b);
      
      float sum_norm = sum.norm();
      float v_norm_squared = v_norm * v_norm;
      
      lambda = sqrt(sum_norm + v_norm_squared);
      
      seg.gamma = (lambda - v_norm) / b_norm;
      seg.gamma = constrain(seg.gamma, 1e-3f, 1.0f);
    } else {
      // Cas dégénéré
      lambda = v_norm;
      seg.gamma = 0.1f;
    }
    
    return lambda;
  }



  void updateQuaternionWithStretchingBending(int i, Vec3 v, Quat b, float lambda) {
    Segment s = segments.get(i);
    
    // Équation 18: q_i = vbe3 + λb
    Vec3 e3 = new Vec3(0, 0, 1);
    //Vec3 v_cross_e3 = v.cross(e3);
    //Quat q_v_cross_e3 = new Quat(0, v_cross_e3.x, v_cross_e3.y, v_cross_e3.z);
    
    // Interprétation 2: v·e3 (dot product) mais comme quaternion
    Vec3 v_dot_e3 = new Vec3(v.x * e3.x, v.y * e3.y, v.z * e3.z);
    Quat q_v_dot_e3 = new Quat(0, v_dot_e3.x, v_dot_e3.y, v_dot_e3.z);
    
    // λb
    Quat q_lambda_b = b.mul(lambda);
    
    // q_i = vbe3 + λb
    // Essayer les deux interprétations:
    //Quat q_new = q_v_cross_e3.add(q_lambda_b); // Version cross product
    Quat q_new = q_v_dot_e3.add(q_lambda_b); // Version dot product
    
    q_new.normalize();
    
    float blend = 0.3f;
    s.q = s.q.slerp(q_new, blend);
    
    if (i < segments.size() - 1) {
      adjustNextPositionForOrientation(i, DT);
    }
  }
  
  // deepseek
  void adjustNextPositionForOrientation(int i, float h) {
    if (i >= segments.size() - 1) return;
    
    Segment current = segments.get(i);
    Segment next = segments.get(i+1);
    
    // Seulement si les deux segments ne sont pas pinned
    if (current.pinned && next.pinned) return;
    
    // Direction selon l'orientation
    Vec3 localDir = new Vec3(0, 0, 1);
    Vec3 worldDir = rotateVectorByQuaternion(current.q, localDir);
    worldDir = worldDir.normalized();
    
    // Position cible
    Vec3 targetPos = current.p_pred.add(worldDir.mul(current.l0));
    
    // Différence
    Vec3 diff = targetPos.sub(next.p_pred);
    float diffLength = diff.length();
    
    // CRITIQUE: Ne corriger QUE SI la différence est modérée
    // Si la différence est trop grande, c'est que l'orientation est déjà très erronée
    float maxAllowedDiff = current.l0 * MAX_ORIENTATION_CORRECTION;
    
    if (diffLength > maxAllowedDiff) {
      // Au lieu de corriger, mieux vaut réinitialiser l'orientation!
      resetOrientationToMatchPositions(i);
      return;
    }
    
    // Correction MINIME
    float stiffness = 0.02f; // TRÈS FAIBLE
    stiffness *= (1.0f / (1.0f + current.k_bt)); // Plus k_bt est grand, MOINS on corrige!
    
    // Application ultra-conservative
    if (!next.pinned) {
      Vec3 correction = diff.mul(stiffness * 0.3f);
      next.p_pred = next.p_pred.add(correction);
    }
    if (!current.pinned) {
      Vec3 correction = diff.mul(stiffness * 0.1f);
      current.p_pred = current.p_pred.sub(correction);
    }
  }

    void resetOrientationToMatchPositions(int i) {
    if (i >= segments.size() - 1) return;
    
    Segment current = segments.get(i);
    Segment next = segments.get(i+1);
    
    // Calculer la direction du segment
    Vec3 segmentDir = next.p_pred.sub(current.p_pred);
    if (segmentDir.length() < 1e-6) return;
    segmentDir = segmentDir.normalized();
    
    // Créer un quaternion qui aligne (0,0,1) avec cette direction
    Vec3 zAxis = new Vec3(0, 0, 1);
    Vec3 axis = zAxis.cross(segmentDir);
    float angle = acos(constrain(zAxis.dot(segmentDir), -1, 1));
    
    if (axis.length() > 1e-6) {
      axis = axis.normalized();
      float sinHalf = sin(angle * 0.5f);
      float cosHalf = cos(angle * 0.5f);
      
      Quat q_target = new Quat(cosHalf, axis.x * sinHalf, axis.y * sinHalf, axis.z * sinHalf);
      
      // Réinitialiser progressivement
      float blend = 0.1f;
      current.q = current.q.slerp(q_target, blend);
      current.q.normalize();
    }
  }
  

  void updatePhiField(int i, float h) {
    Segment seg = segments.get(i);
  
    Quat currentPhi = seg.q.mul(seg.Q_rest.conjugate());
    
    float alpha = 0.1f * h;
    seg.phi = seg.phi.slerp(currentPhi, alpha);
    
    if (seg.phi.norm() > 1.0f) {
      seg.phi.normalize();
    }
  }

  Vec3 rotateVectorByQuaternion(Quat q, Vec3 v) {
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    Vec3 q_xyz = new Vec3(qx, qy, qz);
    Vec3 cross1 = q_xyz.cross(v);
    Vec3 temp = v.mul(qw).add(cross1);
    Vec3 cross2 = q_xyz.cross(temp);
    
    return v.add(cross2.mul(2.0f));
  }

  Vec3 computeStretchingEffectAtPoint(int i) {
    if (i <= 0 || i >= segments.size() - 1) {
      return new Vec3(0, 0, 0);
    }
    
    Segment leftSeg = segments.get(i-1);
    Segment rightSeg = segments.get(i);
    
    // Contribution gauche: -2·k_{i-1}^{SS}·(1/l₀)·(x_i - x_{i-1})
    Vec3 dx_left = segments.get(i).p_pred.sub(segments.get(i-1).p_pred);
    Vec3 v_left = dx_left.mul(-2.0f * leftSeg.k_ss / leftSeg.l0);
    
    // Contribution droite: -2·k_i^{SS}·(1/l₀)·(x_{i+1} - x_i)
    Vec3 dx_right = segments.get(i+1).p_pred.sub(segments.get(i).p_pred);
    Vec3 v_right = dx_right.mul(-2.0f * rightSeg.k_ss / rightSeg.l0);
    
    // Somme des contributions
    return v_left.add(v_right);
  }

  Quat computeBendingEffectAtPoint(int i) {
    if (i <= 0 || i >= segments.size() - 1) {
      return new Quat(0, 0, 0, 0);
    }

    Segment prev = segments.get(i-1);
    Segment current = segments.get(i);
    Segment next = segments.get(i+1);

    // IMPORTANT: Utiliser le phi scalaire, pas le quaternion!
    // φ_{i-1} est sur l'arête entre i-1 et i
    float phi_prev_val = computePhiValue(i-1);
    // φ_{i} est sur l'arête entre i et i+1  
    float phi_next_val = computePhiValue(i);

    // Premier terme: k_{i-1}^{bt}·φ_{i-1}·q_{i-1}^0
    // φ_{i-1}·q_{i-1}^0 = φ_{i-1} * q_{i-1}^0 (multiplication scalaire)
    Quat term1 = prev.Q_rest.mul(phi_prev_val);
    term1 = term1.mul(prev.k_bt);

    // Deuxième terme: k_i^{bt}·φ_{i}·q_i^0
    // φ_{i}·q_i^0 = φ_{i} * q_i^0 (multiplication scalaire)
    Quat term2 = current.Q_rest.mul(phi_next_val);
    term2 = term2.mul(current.k_bt);

    return term1.add(term2);
  }
}