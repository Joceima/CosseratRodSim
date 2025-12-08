class Segment {
  int id;
  Vec3 p;          // Position actuelle
  Vec3 p_pred;     // Position prédite
  Vec3 v;          // Vitesse
  Quat q;          // Orientation (quaternion)
  Quat Q_rest;     // Orientation au repos
  Quat phi;        // Champ φ (déviation)
  float l0;        // Longueur au repos
  float k_ss;      // Raideur stretching/shearing
  float k_bt;      // Raideur bending/twisting
  boolean pinned;  // Est fixé
  float gamma;     // Variable pour λ
  
  // Constructeur
  Segment(int id, Vec3 p, Quat q, Vec3 v, float l0) {
    this.id = id;
    this.p = p.copy();
    this.p_pred = p.copy();
    this.v = v.copy();
    this.q = q.copy();
    this.Q_rest = q.copy();
    this.phi = new Quat(1, 0, 0, 0);
    this.l0 = l0;
    this.k_ss = K_SS;  // Valeur par défaut
    this.k_bt = K_BT;  // Valeur par défaut
    this.pinned = false;
    this.gamma = 0.5f;
  }
  
  // Copie
  Segment copy() {
    Segment copy = new Segment(this.id, this.p, this.q, this.v, this.l0);
    copy.p_pred = this.p_pred.copy();
    copy.Q_rest = this.Q_rest.copy();
    copy.phi = this.phi.copy();
    copy.k_ss = this.k_ss;
    copy.k_bt = this.k_bt;
    copy.pinned = this.pinned;
    copy.gamma = this.gamma;
    return copy;
  }
}