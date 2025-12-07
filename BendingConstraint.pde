class BendingConstraint {
    Triangle triangleA;
    Triangle triangleB;
    Point[] sharedEdge = new Point[2];
    Point oppositeA;
    Point oppositeB;
    float restAngle;
    float stiffness;

    BendingConstraint(Triangle tA, Triangle tB, float stiffness)
    {
        this.triangleA = tA;
        this.triangleB = tB;
        this.stiffness = stiffness;

        findSharedEdgeAndOpposites();

        this.restAngle = computeAngleBetweenNormals(tA.normal, tB.normal);
    }

    float computeAngleBetweenNormals(Vec3 n1, Vec3 n2)
    {
        if(n1.length() == 0 || n2.length() == 0)
        {
            return 0.0f;
        }
        Vec3 normal1 = n1.copy().normalized();
        Vec3 normal2 = n2.copy().normalized();

        float dotProduct = normal1.dot(normal2);
        dotProduct = constrain(dotProduct, -1.0f, 1.0f);
        float angle = acos(dotProduct);
        return angle;
    }

    void findSharedEdgeAndOpposites()
    {
        ArrayList<Point> shared = new ArrayList<Point>();
        Point[] tAPoints = {triangleA.p1, triangleA.p2, triangleA.p3};
        Point[] tBPoints = {triangleB.p1, triangleB.p2, triangleB.p3};
        for(Point pA : tAPoints)
        {
            for(Point pB : tBPoints)
            {
                if(pA == pB && !shared.contains(pA))
                {
                    shared.add(pA);
                }
            }
        }
        sharedEdge[0] = shared.get(0);
        sharedEdge[1] = shared.get(1);
        for(Point p: tAPoints)
        {
            if(p != sharedEdge[0] && p != sharedEdge[1])
            {
                oppositeA = p;
                break;
            }
        }
        for(Point p: tBPoints)
        {
            if(p != sharedEdge[0] && p != sharedEdge[1])
            {
                oppositeB = p;
                break;
            }
        }
    }
}