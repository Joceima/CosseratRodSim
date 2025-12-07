class Point {
    int id;
    Vec3 position;
    Vec3 velocity;
    Vec3 force;
    float mass;
    boolean fixed;

    Point(int id,Vec3 position, Vec3 velocity, float mass, boolean fixed)
    {
        this.position = position;
        this.velocity = velocity;
        this.mass = mass;
        this.fixed = fixed;
    }

    void applyForce(Vec3 f) {
        if (!fixed) {
            this.force.add(f);
        }
    }

    void update(float dt) {
        if (!fixed) {
            // Accélération = Force / Masse
            Vec3 acceleration = force.div( mass);
            
            // Vitesse
            velocity.add(acceleration.mul(dt));
            
            // Position
            position.add(velocity.mul(dt));
            
            // Réinitialiser la force
            force.mul(0.0f);
        }
    }
}