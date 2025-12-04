class Rod {
  ArrayList<Segment> segments;

  Rod() {
    segments = new ArrayList<Segment>();
  }

  void addSegment(int _indice, Vec3 _p, Quat _q, Vec3 _v) {
    segments.add(new Segment(_indice, _p, _q, _v));
  }

  void applyGravity(float h, Vec3 g)
  {
    // pour chaque segment : v+= g*h (sauf pinned)
    for(Segment s : segments)
    {
      if(!s.pinned)
      {
        s.v = s.v.add(g.mul(h));
      }
    }
  }

  void integrateExplicit(float h)
  { 
    // pour chaque segment : p += v*h (sauf pinned)
    for(Segment s : segments)
    {
      if(!s.pinned)
      {
        s.p = s.p.add(s.v.mul(h));
      }
    }
  }

  void applyDamping(float factor) {
    // v *= factor
    for(Segment s : segments)
    {
      s.v = s.v.mul(factor);
    }
  }

  void pinSegment(int id) {
    // segments.get(id).pinned = true; v = 0
    if(id >= 0 && id <this.segments.size())
    {
      this.segments.get(id).pinned = true;
      this.segments.get(id).v = new Vec3(0,0,0);
    }
  }


}
