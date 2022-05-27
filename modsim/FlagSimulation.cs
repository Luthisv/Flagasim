using UnityEngine;
using System.Collections;
using System.Collections.Generic;
public class FlagSimulation : MonoBehaviour
{
  [Header("Cloth initialization arguments")]
  public float width = 1.618f;
  public float height = 1;
  public int cols = 15;
  public int rows = 15;

  [Header("Cloth simulation properties")]
  public float stiffness = 4000f;
  public float damping = 4f;
  public float bendStiffness = 4f;
  public float bendDamping = 1f;
  public float shearStiffness = 10f;
  public float shearDamping = 1f;
  public float airResistance = 0.05f;
  public float windMagnitude = 2800f;
  
  [Header("Toggle forces")]
  public bool gravityOn = true;
  public bool randomForceOn = false;
  public bool structuredOn = true;
  public bool bendForcesOn = true;
  public bool shearForcesOn = true;
  public bool windForceOn = true;
  public bool airForceOn = true;
  
  [Header("Extra features")]
  public bool fixSomePoints = true;
  public bool interSectionLineOn = false;
  public bool collisionOn = false;

  [Header("Cloth texture")]
  public Material mat;

  [Header("Timestep Length")]
  public float integratorTimeStep = 1.0f / 60.0f;


  private int numPoints;

  // Resting horizontal distance between two points in the mesh.
  float dx;
  // Resting vertical distance between two points in the mesh.
  float dy;

  private float timeSinceUpdate;
  private Mesh mesh;
  
  // A 2d-array with additional functionality containg all the MeshPoints
  private MeshPointManager mpManager;

  private GameObject flagPole;


  public Vector3 Gravity = new Vector3(0.0f, -9.81f, 0.0f);
  public Integrator integrator = new RK4();



  private GameObject s1, s2;
  public void Start()
  {
    numPoints = cols*rows;
    dx = width/(cols-1);
    dy = height/(rows-1);

    initializeMesh();    
    setStartPos();

    // Initialize MeshPoints
    mpManager = new MeshPointManager(cols: cols, rows: rows, mesh: mesh);

    
    s1 = GameObject.Find("Sphere1");
    s2 = GameObject.Find("Sphere2");
  }

  // Create the flag's mesh and set the meshes material
  private void initializeMesh() 
  {
    // Set flag material 
    MeshRenderer meshRenderer = gameObject.AddComponent<MeshRenderer>();
    meshRenderer.sharedMaterial = mat;    
    
    // Make meshrenderer cast shadows from both sides of the cloth
    meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.TwoSided;    
    
    // Generate mesh
    MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
    mesh = MeshGenerator.GetMesh(width, height, cols, rows);
    meshFilter.mesh = mesh;
  }


  // Set the position to be at the top of the flag
  private void setStartPos()
  { 
    flagPole = GameObject.Find("Flagpole");
    float flagHeight = flagPole.GetComponent<CapsuleCollider>().height*flagPole.transform.localScale.y;
    float offsetFromCenter = flagHeight/2 - height;
    Vector3 startPos = flagPole.transform.position + new Vector3(0.1f, offsetFromCenter, 0);
    
    this.transform.position = startPos; 
  }

  private void ApplyForces(float timeStep)
  {
    // Clear forces from previous timestep.
    ClearForces();

    // Forces can be toggled from the Unity UI.
    if (gravityOn) ApplyGravityForce();
    if (randomForceOn) ApplyRandomForce();
    if (structuredOn) ApplyStructuralSprings();
    if (bendForcesOn) ApplyBendingForces();
    if (shearForcesOn) ApplyShearForces();
    if (windForceOn) ApplyWindForce();
    if (airForceOn) ApplyAirResistanceForce();
    if (fixSomePoints) FixSomePoints();
  }

  private void ApplyGravityForce()
  {
    foreach (MeshPoint mp in mpManager)
    {
      // The mass of a point is currently set to be a constant unaffected by the number of points in the mesh. 
      // This means that the total mass of the cloth dependes on the amount of points used in the simulation.
      // To make the the total gravity force on the cloth not change based on the number of points we divide by the number of points as a workaround.
      mp.AddForce(Gravity * mp.Mass/numPoints);
    }
  }

  // Purely exists for testing purposes
  private void ApplyRandomForce()
  {
    foreach (MeshPoint mp in mpManager)
    {
      Vector3 force = Random.insideUnitSphere;
      force /= 50;
      mp.AddForce(force);
    }

  }

  private void ClearForces()
  {
    foreach (MeshPoint mp in mpManager)
    {
      mp.ClearForce();
    }
  }

  private void ApplyStructuralSprings()
  {
    // Iterate through rows
    foreach ((MeshPoint p1, MeshPoint p2) in mpManager.IteratePairwise(0))
    {
      ApplySpringForce(p1, p2, dx, damping, stiffness);
    }

    // Iterate through columns
    foreach ((MeshPoint p1, MeshPoint p2) in mpManager.IteratePairwise(1))
    {
      ApplySpringForce(p1, p2, dy, damping, stiffness);
    }
  }

  private void ApplyBendingForces()
  {
    // iterate column-wise.
    foreach ((MeshPoint p1, MeshPoint p2) in mpManager.IteratePairwise(0, 2))
    {
      ApplySpringForce(p1, p2, 2*dx, bendDamping, bendStiffness);
    }

    // Iterate row-wise.
    foreach ((MeshPoint p1, MeshPoint p2) in mpManager.IteratePairwise(1, 2))
    {
      ApplySpringForce(p1, p2, 2*dy, bendDamping, bendStiffness);
    }
  }

  // Calculate the spring force using Hooke's law and apply a damping force based on the velocity.
  private void ApplySpringForce(MeshPoint p1, MeshPoint p2, float rest_dist, float damp_factor, float stiffness)
  {
    Vector3 dist = p2.State.Position - p1.State.Position;
    Vector3 v_rel = p1.State.Velocity - p2.State.Velocity;
    Vector3 F_damp = -damp_factor * v_rel; 
    Vector3 F_p1 = stiffness * (dist.magnitude - rest_dist) * dist.normalized + F_damp;
    Vector3 F_p2 = -F_p1;

    p1.AddForce(F_p1);
    p2.AddForce(F_p2);
  }

  private Vector3 WindForce(Vector3 pos = default(Vector3))
  {
    return new Vector3(1,0,1) * windMagnitude * Mathf.Abs(Mathf.Sin(Time.timeSinceLevelLoad*10));
    
  }

  private void ApplyAirResistanceForce()
  {
    foreach(var triplet in mpManager.IterateTriangles())
    {
      MeshPoint p1,p2,p3;
      (p1,p2,p3) = triplet;

      Vector3 avg_velocity = p1.State.Velocity/3 + p2.State.Velocity/3 + p3.State.Velocity/3;
      float area = trisArea(p1.State.Position, p2.State.Position, p3.State.Position);
      Vector3 normal = trisNormal(p1.State.Position, p2.State.Position, p3.State.Position);
      float v2 = avg_velocity.magnitude * avg_velocity.magnitude;

      Vector3 airForce = - normal * v2 * Vector3.Dot(normal, avg_velocity.normalized) * area * airResistance;
      p1.AddForce(airForce/3);
      p2.AddForce(airForce/3);
      p3.AddForce(airForce/3);
    }
  }

  // Calculate the area of the input triangle using Heron's formula (https://en.wikipedia.org/wiki/Heron%27s_formula)
  float trisArea(Vector3 p1, Vector3 p2, Vector3 p3) 
{
    Vector3 pos1 = p1;
    Vector3 pos2 = p2;
    Vector3 pos3 = p3;

    // Length of the edges of the triangle
    float a = (p1 - p2).magnitude;
    float b = (p1 - p3).magnitude;
    float c = (p2 - p3).magnitude;

    float s = (a+b+c)/2;

    return Mathf.Sqrt(s*(s-a)*(s-b)*(s-c));
}

// from smmflag.wordpress.com
// Calculate the normal of the plane the triangle lies in
Vector3 trisNormal(Vector3 p1, Vector3 p2, Vector3 p3) 
{
    // Two vectors parallel to the plane
    Vector3 v1 = p1-p2;
    Vector3 v2 = p1-p3;

    return Vector3.Cross(v1, v2).normalized;
}

// Gives the MeshPoint triplets of all triangles of the mesh

  private void ApplyWindForce()
  {
    Vector3 wf = WindForce();
    // Apply wind force to all triangles in the mesh.
    foreach(var triplet in mpManager.IterateTriangles())
    {
      MeshPoint p1,p2,p3;
      (p1,p2,p3) = triplet;

      float area = trisArea(p1.State.Position, p2.State.Position, p3.State.Position);
      Vector3 normal = trisNormal(p1.State.Position, p2.State.Position, p3.State.Position);// can take normal from mesh.normals[index]

      // Project the wind force onto the normal of the triangle
      Vector3 wf_projection = Vector3.Dot(normal, wf)*normal;
      
      // Calculate force across triangle's area
      Vector3 force = wf_projection * area;
      
      // Divide the force on the triangle to its vertices
      p1.AddForce(force/3);
      p2.AddForce(force/3);
      p3.AddForce(force/3);
    }
  }

  private void ApplyShearForces()
  {
    float restLen = Mathf.Sqrt(dx*dx + dy*dy);
    foreach ((MeshPoint p1, MeshPoint p2) in mpManager.IterateShearPoints())
    {  
      ApplySpringForce(p1, p2, restLen, shearDamping, shearStiffness);
    }
  }

  // Fixes the position of some points resembling it being connected to the flagpole. 
  private void FixSomePoints()
  {
    for (int i = 0; i < mpManager.Count; i+=cols)
    {
      // Clear speed and force to prevent the points from moving in the integration timestepping
      mpManager[i].ClearForce();
      mpManager[i].ClearSpeed();
    }
  }

  // Make an integration timestep of the simulation using the chosen integration method.
  private void AdvanceSimulation()
  {
    integrator.Advance(mpManager, ApplyForces, integratorTimeStep);
  }

  // Detect and handle collisions with the cloth.
  private void collisionControl()
  {
    foreach(MeshPoint p in mpManager)
    {
      foreach((MeshPoint m1, MeshPoint m2, MeshPoint m3) in mpManager.IterateTriangles())
      {
        // If the point is part of the triangle it cannot intersect it
        if (p == m1 || p == m2 || p == m3) 
        {
          continue;
        }
        
        (Vector3, Vector3, Vector3) triangle = (m1.State.Position, m2.State.Position, m3.State.Position);
        Vector3 intersectPoint;
        if (Intersection.IntersectsTriangle(p.PrevState.Position, p.State.Position, triangle, out intersectPoint))
        {
          // Show the point of intersection
          Vector3 inter = intersectPoint + this.transform.position;
          drawCross(inter, 1/8f);
          
          Vector3 vel = p.State.Velocity; // could also use avg vel between prev and current timestep
          Vector3 prevPos = p.State.Position;
          Vector3 normal = trisNormal(m1.State.Position, m2.State.Position, m3.State.Position);
          float dist = (prevPos - p.PrevState.Position).magnitude;

          // Get new position and direction after collision handling
          (Vector3 pos, Vector3 dir) = Intersection.bounce(prevPos, intersectPoint, normal, dist);
          
          
          float deltaPos = (intersectPoint - pos).magnitude;
          float maxDelta = Mathf.Sqrt(dx*dx + dy*dy)/4;
          if (deltaPos > maxDelta){ // Prevent oscillation.
            pos = intersectPoint + dir * maxDelta;
          }

          // Update the state of the point
          p.State.Position = pos;
          p.State.Velocity = vel.magnitude*dir*1f;

          // Make the triangle move away from the intersecting point
          Vector3 deltaPosDiv3 = (p.State.Position - intersectPoint)/3;
          m1.State.Position -= deltaPosDiv3;
          m2.State.Position -= deltaPosDiv3; 
          m3.State.Position -= deltaPosDiv3;            

          // Make the triangle get a speed directed away from the intersecting point
          Vector3 deltaVelDiv3 = (vel - p.State.Velocity)/3;
          m1.State.Velocity -= deltaVelDiv3;
          m2.State.Velocity -= deltaVelDiv3; 
          m3.State.Velocity -= deltaVelDiv3;  
        }
      }
    }
  }

  private void testBounce() 
  {
    Vector3 start =  s1.transform.position;
    Vector3 end = s2.transform.position;
    foreach((MeshPoint m1, MeshPoint m2, MeshPoint m3) in mpManager.IterateTriangles())
    {
      Vector3 normal = -trisNormal(m1.State.Position, m2.State.Position, m3.State.Position);
      (Vector3, Vector3, Vector3) triangle = (m1.State.Position + transform.position, m2.State.Position + transform.position, m3.State.Position + transform.position);
      Vector3 intersectPoint;
      if (Intersection.IntersectsTriangle(start, end, triangle, out intersectPoint)) 
      {
        (Vector3 pos, Vector3 dir) = Intersection.bounce(start, intersectPoint, normal, (start-end).magnitude);
        Debug.DrawLine(start, intersectPoint, Color.red, integratorTimeStep);
        Debug.DrawLine(pos, intersectPoint, Color.green, integratorTimeStep);
      }
    }

    Debug.DrawLine(start, end, Color.red, integratorTimeStep);
  }

  private void drawCross(Vector3 point, float size = 1f)
  {
    Debug.DrawLine(point + Vector3.up*size, point, Color.blue, integratorTimeStep);
    Debug.DrawLine(point + Vector3.down*size, point, Color.blue, integratorTimeStep);
    Debug.DrawLine(point + Vector3.forward*size, point, Color.yellow, integratorTimeStep);
    Debug.DrawLine(point + Vector3.back*size, point, Color.yellow, integratorTimeStep);
    Debug.DrawLine(point + Vector3.left*size, point, Color.black, integratorTimeStep);
    Debug.DrawLine(point + Vector3.right*size, point, Color.black, integratorTimeStep);
  }

  public void Update()
  {
    bool updatedPos = false;
    timeSinceUpdate += Mathf.Min(Time.deltaTime / integratorTimeStep, 3.0f);
    while (timeSinceUpdate > 1.0f)
    {
      updatedPos = true;
      timeSinceUpdate -= 1.0f;
      AdvanceSimulation();
    }

    // update mesh vertices positions
    if (updatedPos)
    {
      if (interSectionLineOn) testBounce();
      if (collisionOn) collisionControl();

      Vector3[] vertices = mesh.vertices;
      int i = 0;
      foreach (MeshPoint mp in mpManager)
      {
        vertices[i++] = mp.State.Position;
        mp.PrevState = mp.State.Clone();
      }
      mesh.vertices = vertices;
      mesh.RecalculateNormals();
    }
  }
}