class Spring {
    Point pointA;
    Point pointB;
    float restLength;
    float stiffness;

    Spring(Point pointA, Point pointB, float restLength, float stiffness)
    {
        this.pointA = pointA;
        this.pointB = pointB;
        this.restLength = restLength;
        this.stiffness = stiffness;
    }

    void computeForce()
    {
        Vec3 delta = pointB.position.sub(pointA.position);
        float currentLenght = delta.length();
        float stretch = currentLenght - this.restLength;
        delta.normalized();

        float forceMagnetide = - this.stiffness * stretch;
        Vec3 force = delta.mul(forceMagnetide);
        pointA.applyForce(force.mul(-1));
        pointB.applyForce(force);
    }
}