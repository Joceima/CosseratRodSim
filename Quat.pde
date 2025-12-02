class Quat {
  float w, x, y, z;

  Quat(float _w, float _x, float _y, float _z) {
    this.w = _w;
    this.x = _x;
    this.y = _y;
    this.z = _z;
  }

  Quat() {
    this(1, 0, 0, 0); // quaternion identité
  }

  Quat copy() {
    return new Quat(w, x, y, z);
  }

  // multiplication quaternion
  Quat mul(Quat _q) {
    return new Quat(
      w*_q.w - x*_q.x - y*_q.y - z*_q.z,
      w*_q.x + x*_q.w + y*_q.z - z*_q.y,
      w*_q.y - x*_q.z + y*_q.w + z*_q.x,
      w*_q.z + x*_q.y - y*_q.x + z*_q.w
    );
  }

  // rotation d'un vecteur
  Vec3 rotate(Vec3 _v) {
    Quat qv = new Quat(0, _v.x, _v.y, _v.z);
    Quat r = this.mul(qv).mul(this.conjugate());
    return new Vec3(r.x, r.y, r.z);
  }

  Quat conjugate() {
    return new Quat(w, -x, -y, -z);
  }
}
