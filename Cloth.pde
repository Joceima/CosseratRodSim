import java.util.HashSet;
class Cloth {
    Point[] points;
    Spring[] springs;
    Triangle[] triangles;
    BendingConstraint[] bendingConstraints;
    int width, height;
    float spacing;
    float structuralStiffness;
    float bendingStiffness;
    float damping = 0.98f;
    
    Cloth(int width, int height, float spacing, 
          float structuralStiffness, float bendingStiffness) {
        this.width = width;
        this.height = height;
        this.spacing = spacing;
        this.structuralStiffness = structuralStiffness;
        this.bendingStiffness = bendingStiffness;
        
        createClothGrid();
    }
    
    void createClothGrid() {
        // Création des points
        points = new Point[width * height];
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                int index = j * width + i;
                float x = i * spacing;
                float y = j * spacing;
                float z = 0;
                
                // Fixer les points du haut
                boolean fixed = (j == 0 && (i == 0 || i == width-1));
                
                points[index] = new Point(index,
                    new Vec3(x, y, z),
                    new Vec3(0, 0, 0),
                    1.0f,
                    fixed
                );
            }
        }
        
        // Création des ressorts
        createSprings();
        
        // Création des triangles
        createTriangles();
        
        // Création des contraintes de bending
        createBendingConstraints();
    }
    
    void createSprings() {
        ArrayList<Spring> springList = new ArrayList<Spring>();
        
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                int index = j * width + i;
                
                // Ressort horizontal
                if (i < width - 1) {
                    springList.add(new Spring(
                        points[index],
                        points[index + 1],
                        spacing,
                        structuralStiffness
                    ));
                }
                
                // Ressort vertical
                if (j < height - 1) {
                    springList.add(new Spring(
                        points[index],
                        points[index + width],
                        spacing,
                        structuralStiffness
                    ));
                }
                
                // Ressorts diagonaux
                if (i < width - 1 && j < height - 1) {
                    float diagLength = spacing * sqrt(2);
                    springList.add(new Spring(
                        points[index],
                        points[index + width + 1],
                        diagLength,
                        structuralStiffness * 0.3f
                    ));
                }
                
                if (i > 0 && j < height - 1) {
                    float diagLength = spacing * sqrt(2);
                    springList.add(new Spring(
                        points[index],
                        points[index + width - 1],
                        diagLength,
                        structuralStiffness * 0.3f
                    ));
                }
            }
        }
        
        springs = springList.toArray(new Spring[0]);
    }
    
    void createTriangles() {
        ArrayList<Triangle> triangleList = new ArrayList<Triangle>();
        
        for (int j = 0; j < height - 1; j++) {
            for (int i = 0; i < width - 1; i++) {
                int tl = j * width + i;
                int tr = j * width + (i + 1);
                int bl = (j + 1) * width + i;
                int br = (j + 1) * width + (i + 1);
                
                // Deux triangles par cellule
                triangleList.add(new Triangle(points[tl], points[tr], points[bl]));
                triangleList.add(new Triangle(points[tr], points[br], points[bl]));
            }
        }
        
        triangles = triangleList.toArray(new Triangle[0]);
    }
    
    void createBendingConstraints() {
        ArrayList<BendingConstraint> bcList = new ArrayList<BendingConstraint>();
        HashSet<String> processed = new HashSet<String>();
        
        for (int i = 0; i < triangles.length; i++) {
            for (int j = i + 1; j < triangles.length; j++) {
                Triangle tA = triangles[i];
                Triangle tB = triangles[j];
                
                if (shareEdge(tA, tB)) {
                    String key = tA.getKey() + "|" + tB.getKey();
                    if (!processed.contains(key)) {
                        bcList.add(new BendingConstraint(tA, tB, bendingStiffness));
                        processed.add(key);
                    }
                }
            }
        }
        
        bendingConstraints = bcList.toArray(new BendingConstraint[0]);
        println("Contraintes de bending créées: " + bendingConstraints.length);
    }
    
    boolean shareEdge(Triangle t1, Triangle t2) {
        int common = 0;
        Point[] t1p = {t1.p1, t1.p2, t1.p3};
        Point[] t2p = {t2.p1, t2.p2, t2.p3};
        
        for (Point p1 : t1p) {
            for (Point p2 : t2p) {
                if (p1 == p2) common++;
            }
        }
        return common == 2;
    }
    
    // MÉTHODE UPDATE PRINCIPALE
    void update(float dt) {
        for (Point p : points) {
            p.force = new Vec3(0, 0, 0);
        }
     
        for (Spring s : springs) {
            s.computeForce();
        }
        
        for (Triangle t : triangles) {
            t.updateNormal();
        }
        
        computeBendingForces();
    
        Vec3 gravity = new Vec3(0, 0.5f, 0);
        for (Point p : points) {
            if (!p.fixed) {
                p.applyForce(gravity.mul(p.mass));
            }
        }
    
        for (Point p : points) {
            if (!p.fixed) {
                Vec3 acceleration = p.force.div(p.mass);
                
                p.velocity = p.velocity.mul(damping);
                p.velocity = p.velocity.add(acceleration.mul(dt));
                
                p.position = p.position.add(p.velocity.mul(dt));
                
                if (p.position.y > height * spacing) {
                    p.position.y = height * spacing;
                    p.velocity.y *= -0.3f;
                }
            }
        }
    }
    
    void computeBendingForces() {
        for (BendingConstraint bc : bendingConstraints) {
            // Angle courant
            float currentAngle = computeAngleBetweenNormals(
                bc.triangleA.normal,
                bc.triangleB.normal
            );
            
            // Différence d'angle
            float angleDiff = currentAngle - bc.restAngle;
            
            // Force proportionnelle à la différence
            float forceMagnitude = -bc.stiffness * angleDiff;
            
            // Appliquer les forces aux 4 points
            applySimpleBendingForce(bc, forceMagnitude);
        }
    }
    
    float computeAngleBetweenNormals(Vec3 n1, Vec3 n2) {
        float dot = n1.dot(n2);
        float len1 = n1.length();
        float len2 = n2.length();
        
        if (len1 < 0.0001f || len2 < 0.0001f) {
            return 0.0f;
        }
        
        dot /= (len1 * len2);
        dot = constrain(dot, -1.0f, 1.0f);
        return acos(dot);
    }
    
    void applySimpleBendingForce(BendingConstraint bc, float magnitude) {
        // Centre de l'arête partagée
        Vec3 edgeCenter = bc.sharedEdge[0].position
            .add(bc.sharedEdge[1].position)
            .mul(0.5f);
        
        // Direction de l'arête
        Vec3 edgeDir = bc.sharedEdge[1].position
            .sub(bc.sharedEdge[0].position)
            .normalized();
        
        // Vecteur du centre au point opposé A
        Vec3 toOppositeA = bc.oppositeA.position.sub(edgeCenter);
        // Composante perpendiculaire à l'arête
        float projA = toOppositeA.dot(edgeDir);
        Vec3 perpendicularA = toOppositeA.sub(edgeDir.mul(projA));
        
        // Même chose pour B
        Vec3 toOppositeB = bc.oppositeB.position.sub(edgeCenter);
        float projB = toOppositeB.dot(edgeDir);
        Vec3 perpendicularB = toOppositeB.sub(edgeDir.mul(projB));
        
        // Normaliser les directions
        if (perpendicularA.length() > 0.0001f) {
            perpendicularA = perpendicularA.normalized();
        }
        if (perpendicularB.length() > 0.0001f) {
            perpendicularB = perpendicularB.normalized();
        }
        
        // Appliquer les forces
        Vec3 forceA = perpendicularA.mul(magnitude * 0.1f);
        Vec3 forceB = perpendicularB.mul(-magnitude * 0.1f);
        
        bc.oppositeA.applyForce(forceA);
        bc.oppositeB.applyForce(forceB);
    }


     void drawSimple() {
        // Option 1: Dessiner les triangles avec remplissage uni
        drawTrianglesSolid();
        
        // Option 2: Dessiner les arêtes (décommenter pour activer)
        // drawEdges();
        
        // Option 3: Dessiner les points (décommenter pour activer)
        // drawPoints();
    }
    
    // Dessiner les triangles avec une couleur unique
    void drawTrianglesSolid() {
        fill(200, 220, 255); // Couleur bleu clair
        stroke(150, 170, 220); // Contour plus foncé
        strokeWeight(1);
        
        beginShape(TRIANGLES);
        for (Triangle t : triangles) {
            vertex(t.p1.position.x, t.p1.position.y, t.p1.position.z);
            vertex(t.p2.position.x, t.p2.position.y, t.p2.position.z);
            vertex(t.p3.position.x, t.p3.position.y, t.p3.position.z);
        }
        endShape();
    }
    
    // Dessiner seulement les arêtes (mode fil de fer)
    void drawEdges() {
        noFill();
        stroke(100, 120, 180);
        strokeWeight(1.5);
        
        // Dessiner chaque arête de chaque triangle
        for (Triangle t : triangles) {
            beginShape(LINES);
            vertex(t.p1.position.x, t.p1.position.y, t.p1.position.z);
            vertex(t.p2.position.x, t.p2.position.y, t.p2.position.z);
            
            vertex(t.p2.position.x, t.p2.position.y, t.p2.position.z);
            vertex(t.p3.position.x, t.p3.position.y, t.p3.position.z);
            
            vertex(t.p3.position.x, t.p3.position.y, t.p3.position.z);
            vertex(t.p1.position.x, t.p1.position.y, t.p1.position.z);
            endShape();
        }
    }
    
    // Dessiner seulement les points
    void drawPoints() {
        noStroke();
        
        // Points normaux en bleu
        fill(100, 150, 255);
        for (Point p : points) {
            if (!p.fixed) {
                ellipse(p.position.x, p.position.y, 3, 3);
            }
        }
        
        // Points fixes en rouge
        fill(255, 50, 50);
        for (Point p : points) {
            if (p.fixed) {
                ellipse(p.position.x, p.position.y, 5, 5);
            }
        }
    }
    
    // Dessiner en mode "grille" (ressorts visibles)
    void drawGrid() {
        noFill();
        stroke(180, 200, 255, 150);
        strokeWeight(1);
        
        // Dessiner les ressorts horizontaux et verticaux
        for (Spring s : springs) {
            line(s.pointA.position.x, s.pointA.position.y, s.pointA.position.z,
                 s.pointB.position.x, s.pointB.position.y, s.pointB.position.z);
        }
        
        // Dessiner les points
        drawPoints();
    }
}