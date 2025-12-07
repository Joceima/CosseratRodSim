class Rod {
  ArrayList<Segment> segments = new ArrayList<Segment>();

  //Rod(int count, Vec3 origin, Vec3 dir, float l0) {
  //  for (int i = 0; i < count; i++) {
  //    Vec3 p = origin.add(dir.mul(i * l0));
  //    Quat q = new Quat(1, 0, 0, 0); // Orientation par défaut
  //    Vec3 v0 = new Vec3(0, 0, 0);
  //    addSegment(i, p, q, v0, l0);
  //  }
  //}

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

  void applyGravity(float h, Vec3 gravity)
  {
    for (Segment s : segments) {
      if (!s.pinned) {
        s.p_pred = s.p.add(s.v.mul(h));
      } else {
        s.p_pred = s.p.copy();
        s.v = new Vec3(0, 0, 0);
      }
    }     
    for (Segment s : segments) {
      if (!s.pinned) {
        s.p_pred.y += 0.5f * gravity.y * h * h; // Formule de chute libre: y = ½gt²
      }
    }
  }
  void step(float h) {
    // === ÉTAPE 1: Prédiction des positions ===
    
    applyGravity(h, gravity);
    
    // === ÉTAPE 2: Résolution des contraintes sur p_pred ===
    for (int iter = 0; iter < 10; iter++) {
      for (int i = 0; i < segments.size() - 1; i++) {
        solveDistanceConstraintOnPredicted(i);
      }
    }

    for(int i = 1; i < segments.size() - 1; i++)
    {
      Vec3 v = computeStretchingEffectAtPoint(i);
      Quat b = computeBendingEffectAtPoint(i);
      float lambda = computeBendingLambda(i,v,b);
      updateQuaternionWithBending(i,v,b,lambda);

      updatePhiField(i,v,b,lambda,h);  
    }

    // === ÉTAPE 3: Mise à jour des vitesses et positions ===
    for (Segment s : segments) {
      if (!s.pinned) {
        s.v = s.p_pred.sub(s.p).mul(1.0f / h);
        s.v = s.v.mul(0.98f);
        s.p = s.p_pred.copy();
      }
    }
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


  Quat computeBendingEffectAtPoint(int i) {
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
    
    return b;

  }

  // v = −2·k_ss·(1/l0)·(x_{i+1} - x_i)
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

   return v_left.add(v_right).mul(FACTOR_SS);
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

    //lambda = constrain(lambda, 0.001f, 10.0f);
    s.gamma = constrain(s.gamma, 0.001f, 1.0f);

    return lambda;
  }

  void updateQuaternionWithBending(int i, Vec3 v, Quat b, float lambda) {
    Segment s = segments.get(i);
    Vec3 old_p_pred = s.p_pred.copy();
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
    //s.q = q_new;

    if(i < segments.size() -1)
    {
      Segment next = segments.get(i+1);
      Vec3 localDirection = new Vec3(0,0,1);
      Vec3 wolrdDirection = rotateVectorByQuaternion(s.q, localDirection);
      Vec3 targetPos = old_p_pred.add(wolrdDirection.normalized().mul(s.l0));
      Vec3 correction = targetPos.sub(next.p_pred);

      float stiffnessFactor = s.k_bt * 0.001f;
      float maxCorrection = s.l0 * 0.9f;

      if(correction.length() > maxCorrection)
      {
        correction = correction.normalized().mul(maxCorrection);
      }
      if (!next.pinned) {
          next.p_pred = next.p_pred.sub(correction.mul(stiffnessFactor));
      }
      if( !s.pinned) {
        s.p_pred = s.p_pred.sub(correction.mul(stiffnessFactor));
      }
    }
  }

  Vec3 rotateVectorByQuaternion(Quat q, Vec3 v) {
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    float vx = v.x, vy = v.y, vz = v.z;
    
    Vec3 q_xyz = new Vec3(qx, qy, qz);
    Vec3 cross1 = q_xyz.cross(v);
    Vec3 temp = v.mul(qw).add(cross1);
    Vec3 cross2 = q_xyz.cross(temp);
    
    return v.add(cross2.mul(2.0f));
  }
  // je ne savais pas comment mettre à jour phi, donc j'ai demandé à DeepSeek de m'assister pour ce code
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
