 using UnityEngine;

 public class Intersection
 {
  /*
    Checks for intersection between a line and triangle and returns the point of intersection if there is any.
    Uses the Möller-Trumbore algorithm.
    
    @start      start point of the line checked
    @end        end point of the line checked
    @T          the vertices of the triangle
    @intersect  the point of the intersection
  */
  public static bool IntersectsTriangle(Vector3 start, Vector3 end, (Vector3 A, Vector3 B, Vector3 C) T, out Vector3 intersect)
  {
    Vector3 A = T.A;
    Vector3 B = T.B;
    Vector3 C = T.C;
    Vector3 D = end - start;
    Vector3 O = start;
    intersect = Vector3.zero;
    

    (Vector3, Vector3, Vector3) matrix = (-D, (B-A), (C-A)); 
    Vector3 sol = solve(matrix, O-A);
    float t, u, v;
    t = sol.x; u = sol.y; v = sol.z;

    // Check if the point is outside the triangle 
    if (u < 0 || v < 0 || (v+u) > 1 || t < 0 || t > 1) 
    {
      return false;
    } else if (float.IsNaN(t) || float.IsNaN(u) || float.IsNaN(v)) // Check if the line is nearly parallel to the triangle 
    {
      return false;
    } else {
      intersect = O + D*t;
      return true;
    }
  }

  /*
    Returns the solution to a 3x3 linear equation system m*x = b.
    Uses Cramer's rule

    @m  the 3x3 matrix
    @b  the right hand side of the equation mx = b
  */
  private static Vector3 solve( (Vector3 A, Vector3 B, Vector3 C) m, Vector3 b)
  {
    // Calculate necessary determinants for Cramer's rule using the scalar triple product.
    float M  = -Vector3.Dot(Vector3.Cross(m.A, m.C), m.B);
    float Mx = -Vector3.Dot(Vector3.Cross(b, m.C), m.B);
    float My = -Vector3.Dot(Vector3.Cross(m.A, m.C), b);
    float Mz = -Vector3.Dot(Vector3.Cross(m.A, b), m.B);
    /*if (Mathf.Abs(M) < 1e-8f){
      Debug.Log("M is: " + M);
    }*/
    
    float x = Mx / M;
    float y = My / M;
    float z = Mz / M;
    
    return new Vector3(x,y,z);
  }

  /*
    Returns the new position and direction after a point has bounced off a wall.

    @start      position prior to the bounce
    @intersect  position of intersection between line and wall
    @normal     normal of the wall
    @dist       distance the point should travel in total (distance before bounce + distance after bounce)
  */
  public static (Vector3 pos, Vector3 dir) bounce(Vector3 start, Vector3 intersect, Vector3 normal, float dist)
  {
    Vector3 v = -intersect+start;
    Vector3 v_projection = Vector3.Dot(v, normal)*normal;
    Vector3 direction = (2*v_projection - v).normalized; // reflection formula
    Vector3 pos = direction*(dist-v.magnitude);
    return (pos, direction);
  }
}