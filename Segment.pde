class Segment {
  int indice;
  Vec3 p;
  Quat q;
  Vec3 v;
  boolean pinned;
  float l0;
  float k_ss = 10.0f;
  float k_bt = 1.0f;

  // fields used by bending code (kept here)
  Vec3 tau = new Vec3(0,0,0);
  Vec3 omega = new Vec3(0,0,0);

  Segment(int _indice, Vec3 _p, Quat _q, Vec3 _v, float _l0) {
    indice = _indice;
    p = _p.copy();
    q = _q.copy();
    v = _v.copy();
    pinned = false;
    l0 = _l0;
  }
}
