class Segment {
  int indice;
  Vec3 p;
  Quat q;
  Vec3 v;
  boolean pinned;
  float l0;
  float k_ss = K_SS;  // Réduit de 10.0 à 0.1
  float k_bt = K_BT; 

  Vec3 tau = new Vec3(0,0,0);
  Vec3 omega = new Vec3(0,0,0);
  Quat lambda = new Quat(0,0,0,0);
  float gamma = GAMMA;

  Quat Q_rest;
  Quat phi = PHI;

  Vec3 p_pred = new Vec3(0,0,0); 

  int i;
  int j;

  Vec3 bending_correction = new Vec3(0, 0, 0);  // Correction due au bending
  Vec3 p_without_bending = new Vec3(0, 0, 0);

  Segment(int _indice, Vec3 _p, Quat _q, Vec3 _v, float _l0) {
    indice = _indice;
    p = _p.copy();
    q = _q.copy();
    v = _v.copy();
    pinned = false;
    l0 = _l0;
  }
}