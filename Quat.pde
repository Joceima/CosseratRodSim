class Quat {
  float w,x,y,z;
  Quat() { w = 1; x = y = z = 0; }
  Quat(float _w,float _x,float _y,float _z) { w=_w; x=_x; y=_y; z=_z; }

  Quat copy() { return new Quat(w,x,y,z); }

  Quat mul(Quat q) {
    return new Quat(
      w*q.w - x*q.x - y*q.y - z*q.z,
      w*q.x + x*q.w + y*q.z - z*q.y,
      w*q.y - x*q.z + y*q.w + z*q.x,
      w*q.z + x*q.y - y*q.x + z*q.w
    );
  }

  Quat add(Quat q)
  {
    return new Quat(w + q.w, x + q.x, y + q.y, z + q.z);
  }

  Vec3 mul(Vec3 v) {
    // Multiplication d'un quaternion par un vecteur : q * v * q^-1
    // Pour une rotation pure (q est unitaire), q^-1 = q.conjugate()
    
    // Convertir le vecteur en quaternion pur (w=0)
    Quat v_quat = new Quat(0, v.x, v.y, v.z);
    
    // Rotation: q * v * q.conjugate()
    Quat q_conj = conjugate();
    Quat result = this.mul(v_quat).mul(q_conj);
    
    // Retourner la partie vectorielle
    return new Vec3(result.x, result.y, result.z);
  }

  Quat sub(Quat q)
  {
    return new Quat(w - q.w, x - q.x, y - q.y, z - q.z);
  }

  Quat mul(float s) {
    return new Quat(w * s, x * s, y * s, z * s);
  }

  Quat conjugate() { return new Quat(w, -x, -y, -z); }

  float normalizeFloat() 
  {
    float norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 1e-10f) {
      w /= norm;
      x /= norm;
      y /= norm;
      z /= norm;
    } else {
      w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
    }
    return norm;
  }

  float norm() 
  {
    return sqrt(w*w + x*x + y*y + z*z);
  }

  void normalize() 
  {
    float n = sqrt(w*w + x*x + y*y + z*z);
    if (n < 1e-8f) { w = 1; x = y = z = 0; return; }
    w /= n; x /= n; y /= n; z /= n;
  }

  Quat normalized() {
    Quat q = copy(); q.normalize(); return q;
  }


  Quat slerp(Quat target, float t) {
    t = constrain(t, 0.0f, 1.0f);
    
    // Cosinus de l'angle entre les quaternions
    float cosHalfTheta = w*target.w + x*target.x + y*target.y + z*target.z;
    
    // Si cosHalfTheta < 0, on inverse pour prendre le chemin le plus court
    if (cosHalfTheta < 0.0f) {
      target = target.mul(-1.0f);
      cosHalfTheta = -cosHalfTheta;
    }
    
    // Si les quaternions sont très proches, interpolation linéaire
    if (cosHalfTheta > 0.9999f) {
      return new Quat(
        w + (target.w - w) * t,
        x + (target.x - x) * t,
        y + (target.y - y) * t,
        z + (target.z - z) * t
      ).normalized();
    }
    
    // Sinon, SLERP
    float halfTheta = acos(cosHalfTheta);
    float sinHalfTheta = sqrt(1.0f - cosHalfTheta*cosHalfTheta);
    
    // Évite la division par zéro
    if (abs(sinHalfTheta) < 0.001f) {
      return this.copy();
    }
    
    float ratioA = sin((1.0f - t) * halfTheta) / sinHalfTheta;
    float ratioB = sin(t * halfTheta) / sinHalfTheta;
    
    return new Quat(
      w * ratioA + target.w * ratioB,
      x * ratioA + target.x * ratioB,
      y * ratioA + target.y * ratioB,
      z * ratioA + target.z * ratioB
    );
  }

  // build quaternion from axis/angle (instance-style to avoid static)
  Quat fromAxisAngle(Vec3 axis, float angle) {
    Vec3 a = axis.normalized();
    float half = angle * 0.5f;
    float s = sin(half);
    return new Quat(cos(half), a.x*s, a.y*s, a.z*s);
  }

  // instance lookAt: returns a quaternion that rotates +Z (0,0,1) to dir
  Quat lookAt(Vec3 dir) {
    Vec3 f = dir.normalized();
    if (f.length() < 1e-8) return new Quat();
    Vec3 up = new Vec3(0,1,0);
    if (abs(f.dot(up)) > 0.999f) up = new Vec3(1,0,0);
    Vec3 right = up.cross(f).normalized();
    Vec3 newUp = f.cross(right);

    float m00 = right.x, m01 = right.y, m02 = right.z;
    float m10 = newUp.x, m11 = newUp.y, m12 = newUp.z;
    float m20 = f.x, m21 = f.y, m22 = f.z;

    float t = m00 + m11 + m22;
    Quat q = new Quat();
    if (t > 0) {
      float s = sqrt(t + 1.0f) * 2.0f;
      q.w = 0.25f * s;
      q.x = (m21 - m12) / s;
      q.y = (m02 - m20) / s;
      q.z = (m10 - m01) / s;
    } else {
      // fallback: choose largest diagonal
      if (m00 > m11 && m00 > m22) {
        float s = sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
      } else if (m11 > m22) {
        float s = sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
      } else {
        float s = sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
      }
    }
    q.normalize();
    return q;
  }
}
