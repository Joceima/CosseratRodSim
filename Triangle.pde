import java.util.Arrays;
class Triangle {
   Point p1;
   Point p2;
   Point p3;

   Vec3 normal;

   Triangle(Point p1, Point p2, Point p3)
   {
    this.p1 = p1;
    this.p2 = p2;
    this.p3 = p3;
    updateNormal();

   }

   void updateNormal()
   {
    Vec3 v1 = p2.position.sub(p1.position);
    Vec3 v2 = p3.position.sub(p1.position);
    this.normal = v1.cross(v2);
    this.normal.normalized();
   }

   String getKey() {
        int[] ids = {p1.id, p2.id, p3.id};
        Arrays.sort(ids);
        return ids[0] + "_" + ids[1] + "_" + ids[2];
    }
}