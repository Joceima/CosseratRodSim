// Segments parameters
float K_SS = 0.05f; // 0.03f il faut augmenter la valeur pour un élastique très fort
float K_BT = 10.0f;
float FACTOR_SS = 0.5f;
float GAMMA = 0.25f;
Quat PHI = new Quat(0, 0.01f, 0, 0);
Vec3 gravity = new Vec3(0, 3000, 0);

// Simulation parameters
float DT = 0.016;
int NB_OF_POINTS = 10;
int SUBSTEPS = 4;

int ROWS = 4;
int COLS = 4;
float L0 = 1;
float DX = 5;
float DY = 5;

float EPS_3 = 0.001;
float EPS_6 = 0.000001;