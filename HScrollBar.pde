// ============ CLASSE HSCROLLBAR ============
class HScrollbar {
  int swidth, sheight;    // width and height of bar
  float xpos, ypos;       // x and y position of bar
  float spos, newspos;    // x position of slider
  float sposMin, sposMax; // max and min values of slider
  int loose;              // how loose/heavy
  boolean over;           // is the mouse over the slider?
  boolean locked;
  float ratio;

  HScrollbar (float xp, float yp, int sw, int sh, int l) {
    swidth = sw;
    sheight = sh;
    int widthtoheight = sw - sh;
    ratio = (float)sw / (float)widthtoheight;
    xpos = xp;
    ypos = yp-sheight/2;
    spos = xpos + swidth/2 - sheight/2;
    newspos = spos;
    sposMin = xpos;
    sposMax = xpos + swidth - sheight;
    loose = l;
  }

  void update() {
    if (overEvent()) {
      over = true;
    } else {
      over = false;
    }
    if (firstMousePress && over) {
      locked = true;
    }
    if (!mousePressed) {
      locked = false;
    }
    if (locked) {
      newspos = constrain(mouseX-sheight/2, sposMin, sposMax);
    }
    if (abs(newspos - spos) > 0.1) {
      spos = spos + (newspos-spos)/loose;
    }
  }

  float constrain(float val, float minv, float maxv) {
    return min(max(val, minv), maxv);
  }

  boolean overEvent() {
    if (mouseX > xpos && mouseX < xpos+swidth &&
      mouseY > ypos && mouseY < ypos+sheight) {
      return true;
    } else {
      return false;
    }
  }

  void display() {
    noStroke();
    
    // Fond de la barre
    fill(100, 100, 100, 150);
    rect(xpos, ypos, swidth, sheight, 3);
    
    // Curseur
    if (over || locked) {
      fill(200, 220, 255);
    } else {
      fill(150, 180, 255);
    }
    rect(spos, ypos, sheight, sheight, 3);
    
    // Ligne de repère au centre
    stroke(255, 200);
    strokeWeight(1);
    line(xpos + swidth/2, ypos, xpos + swidth/2, ypos + sheight);
    noStroke();
  }

  float getPos() {
    // Retourne une valeur entre 0 et 1
    return map(spos, sposMin, sposMax, 0, 1);
  }
  
  void setPos(float pos) {
    // Positionne le curseur selon une valeur entre 0 et 1
    spos = map(constrain(pos, 0, 1), 0, 1, sposMin, sposMax);
    newspos = spos;
  }
}
// ============ FIN DE HSCROLLBAR ============