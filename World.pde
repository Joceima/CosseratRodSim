int substeps = 4; 
int rows = 4;
int cols = 4;
float l0 = 1;
float dx = 5;
float dy = 5;
class World {
  Rod rod;


  World() {
    rod = new Rod();
  }

   void update() 
  {
    float sub_dt = DT / SUBSTEPS;
    
    for (int i = 0; i < SUBSTEPS  ; i++) {
      rod.step(sub_dt);
    }

  }


 
  void drawPoints() {
    noStroke();
    fill(255, 180, 0);
    for (Segment s : rod.segments) {
      pushMatrix();
      translate(s.p.x, s.p.y, s.p.z);
      sphere(4);
      popMatrix();
    }
  }

  void drawRod() {
    stroke(255);
    strokeWeight(3);
    for (int i = 0; i < rod.segments.size() - 1; i++) {
      Segment a = rod.segments.get(i);
      Segment b = rod.segments.get(i + 1);
      line(a.p.x, a.p.y, a.p.z, b.p.x, b.p.y, b.p.z);
    }
  }


}
