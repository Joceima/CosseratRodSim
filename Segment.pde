class Segment {
  int indice;
  Vec3 p;
  Quat q;
  Vec3 v;
  boolean pinned;
  float l0;
  float k_ss = 1.3f;  // Réduit de 10.0 à 0.1
  float k_bt = 0.05f; // Réduit aussi k_bt

  // fields used by bending code (kept here)
  Vec3 tau = new Vec3(0,0,0);
  Vec3 omega = new Vec3(0,0,0);
  Quat lambda = new Quat(0,0,0,0);
  float gamma = 0.1;

  Quat Q_rest;
  Quat phi = new Quat(0,0.01f,0,0);

  Vec3 p_pred = new Vec3(0,0,0); 

  Segment(int _indice, Vec3 _p, Quat _q, Vec3 _v, float _l0) {
    indice = _indice;
    p = _p.copy();
    q = _q.copy();
    v = _v.copy();
    pinned = false;
    l0 = _l0;
  }
}