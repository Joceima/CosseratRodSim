class Cloth
{
    Point[] points;
    Spring[] springs;
    int width;
    int height;
    float spacing;
    float stiffness;

    void createClothgrid(int width, int height, float spacing, float stiffness)
    {
        this.width = width;
        this.height = height;
        this.spacing = spacing;
        this.stiffness = stiffness;
        // initialisation de la grille
        points = new Point[width * height];
        ArrayList<Spring> springList = new ArrayList<Spring>();
        for(int j = 0; j < height; j++)
        {
            for(int i = 0; i < width; i++)
            {
                int index = j * width +i;
                if(i == 0) // fixe la première ligne
                {
                    points[index] = new Point(
                        new Vec3(i * spacing, j * spacing, 0.0f),
                        new Vec3(0.0f, 0.0f, 0.0f),
                        1.0f,
                        true
                    );
                }
                points[index] = new Point(
                        new Vec3(i * spacing, j * spacing, 0.0f),
                        new Vec3(0.0f, 0.0f, 0.0f),
                        1.0f,
                        false
                    );
            }
        }

        // initialisation de springs
        for(int j = 0; j < height; j++)
        {
            for(int i = 0; i < width; i++)
            {
                int index = j * width + i;
                
                // Ressort horizontal
                if(i < width - 1)
                {
                    Spring spring = new Spring(
                        points[index],
                        points[index + 1],
                        spacing,
                        stiffness
                    );
                    springList.add(spring);
                }

                // Ressort vertical
                if(j < height - 1)
                {
                    Spring spring = new Spring(
                        points[index],
                        points[index + width],
                        spacing,
                        stiffness
                    );
                    springList.add(spring);
                }

                // Ressort diagonal 1 (\) - correction de la formule
                if(i < width - 1 && j < height - 1)
                {
                    float diagonalLength = spacing * sqrt(2);
                    Spring spring = new Spring(
                        points[index],
                        points[index + width + 1],
                        diagonalLength,
                        stiffness * 0.3f // Plus faible pour les diagonales
                    );
                    springList.add(spring);
                }
                
                // Ressort diagonal 2 (/)
                if(i > 0 && j < height - 1)
                {
                    float diagonalLength = spacing * sqrt(2);
                    Spring spring = new Spring(
                        points[index],
                        points[index + width - 1],
                        diagonalLength,
                        stiffness * 0.3f
                    );
                    springList.add(spring);
                }
            }
        }
        
        // Convertir ArrayList en tableau
        springs = new Spring[springList.size()];
        springs = springList.toArray(springs);
    }   
}