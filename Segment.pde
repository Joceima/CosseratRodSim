class Segment {

    int indice;
    Vec3 p;// position 
    Quat q; // orientation
    //float m; // masse
    //float l0; // longueur au repos
    //float radius;
    //float k_ss; // coef de stretching/shear
    //float k_bt; // coef de bending/twist
    //float lambda; // variable auxilière
    //float gamma; // variable auxilière intermédiaire pour rester dans la borne

    Vec3 extForce;
    Vec3 tau_ext;

    Segment(int _indice, Vec3 _p, Quat _q) {
        this.p = _p;
        this.q = _q;
    }
    //Segment(Vec3 p_, Vec3 _q, float m_, float l0_, float radius_)
    //{
    //    q = new Quat();
    //    p = new Vec3();
    //    m = m_;
    //    l0 = l0_;
    //    radius = radius_;
    //    k_ss = 1.0f;
    //    k_bt = 1.0f;
    //    lambda = 0.001f;
    //    gamma = 0.0f;
    //    extForce = new Vec3();
    //    tau_ext = new Vec3();
    //}

}