class Vec3 {
  float x, y, z;

  Vec3(float x, float y, float z) 
  {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  Vec3() 
  {
    this(0, 0, 0);
  }

  Vec3 copy() 
  {
    return new Vec3(x, y, z);
  }

  Vec3 add(Vec3 v) 
  {
    return new Vec3(x + v.x, y + v.y, z + v.z);
  }

  Vec3 sub(Vec3 v) 
  {
    return new Vec3(x - v.x, y - v.y, z - v.z);
  }

  Vec3 mul(float s) 
  {
    return new Vec3(x * s, y * s, z * s);
  }

  Vec3 div(float s)
  {
    return new Vec3 (x/s, y/s, z/s);
  }


  float dot(Vec3 v) 
  {
    return x*v.x + y*v.y + z*v.z;
  }


  Vec3 cross(Vec3 v) 
  {
    return new Vec3(
      y*v.z - z*v.y,
      z*v.x - x*v.z,
      x*v.y - y*v.x
    );
  }

  float length() 
  {
    return sqrt(x*x + y*y + z*z);
  }

  Vec3 normalized() 
  {
    float len = length();
    if (len < ESP_8) return new Vec3(0, 0, 0);
    return new Vec3(x / len, y / len, z / len);
  }

  Quat mul(Quat q) {
    float w_result = -this.dot(new Vec3(q.x, q.y, q.z)); // -v1·qv
    Vec3 v_result = new Vec3(q.w * this.x, q.w * this.y, q.w * this.z)  // qw*v1
                    .add(this.cross(new Vec3(q.x, q.y, q.z))); // + v1×qv

    return new Quat(w_result, v_result.x, v_result.y, v_result.z);
  }

}
