class Rod {
  ArrayList<Segment> segments;

  Rod() {
    segments = new ArrayList<Segment>();
  }

  void addSegment(int _indice, Vec3 _p, Quat _q) {
    segments.add(new Segment(_indice, _p, _q));
  }
}
