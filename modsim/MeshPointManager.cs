using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


// A helper class to more conveniently iterate through the vertices and triangles of a mesh.
public class MeshPointManager
{
  public MeshPoint[,] meshPoints;
  public int cols;
  public int rows;
  public int Count;

  public MeshPoint[] triangles;


  public MeshPointManager(int cols, int rows, Mesh mesh)
  {
    this.cols = cols;
    this.rows = rows;
    Count = cols*rows; 
    meshPoints = new MeshPoint[rows, cols];
    triangles = new MeshPoint[mesh.triangles.Length];
    BuildFromVertices(mesh.vertices);
    buildFromTriangles(triangles, mesh.triangles);
  }

  // Given an array of vertices create an array of MeshPoints with the same positions. 
  private void BuildFromVertices(Vector3[] vertices)
  {
    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols; x++)
      {
        Vector3 pos = vertices[x + y * cols];
        meshPoints[y, x] = new MeshPoint(pos, Vector3.zero);
      }
    }
  }

  // Store each triangle in the mesh as three consecutive points in the input array
  private void buildFromTriangles(MeshPoint[] triangles, int[] meshTriangles)
  {
    for(int i = 0; i < meshTriangles.Length; i+=3)
    {
      MeshPoint p1, p2, p3;
      int i1,i2,i3;
      // Indices of vertexes. 
      i1 = meshTriangles[i];
      i2 = meshTriangles[i+1];
      i3 = meshTriangles[i+2];
      
      // Meshpoints/vertexes of the triangle.
      p1 = this[i1];
      p2 = this[i2];
      p3 = this[i3];
      
      // Adding the vertexes of the triangle.
      triangles[i] = p1;
      triangles[i+1] = p2;
      triangles[i+2] = p3;
    }
  }

  public void RotatePoints()
  {
    Vector3 a = this[Count-1].State.Position;
    foreach(MeshPoint p in meshPoints)
    { 
      p.State.Position = Quaternion.AngleAxis(90f, Vector3.right) * p.State.Position;
    }
  }

  public MeshPoint this[int y, int x]
  {
    get{ return meshPoints[y, x]; }
    set { meshPoints[y, x] = value; }
  }

  public MeshPoint this[int i]
  {
    get
    { 
      int y = i / cols;
      int x = i % cols;
      return meshPoints[y, x];
    }

    set
    {
      int y = i / cols;
      int x = i % cols;
      meshPoints[y, x] = value;
    }

  }

  public IEnumerator GetEnumerator()
  {
    for (int y = 0; y < meshPoints.GetLength(0); y++)
    {
      for (int x = 0; x < meshPoints.GetLength(1); x++)
      {
        yield return meshPoints[y, x];
      }
    }
  }

  public IEnumerable<(MeshPoint, MeshPoint)> IteratePairwise(int axis, int stepSize = 1)
  {
    if (axis == 0)
    {
      for (int y = 0; y < meshPoints.GetLength(0); y++)
      {
        for (int x = 0; x < meshPoints.GetLength(1) - stepSize; x++)
        {
          yield return (meshPoints[y, x], meshPoints[y, x + stepSize]);
        }
      }
    }
    else
    {
      for (int x = 0; x < meshPoints.GetLength(1); x++)
      {
        for (int y = 0; y < meshPoints.GetLength(0) - stepSize; y++)
        {
          yield return (meshPoints[y, x], meshPoints[y + stepSize, x]);
        }
      }
    }
  }

  public IEnumerable<(MeshPoint, MeshPoint)> IterateShearPoints()
  {
    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols-1; x++)
      {
        MeshPoint p1, p2;
        p1 = meshPoints[y,x];
        if(y > 0) 
        {
          p2 = meshPoints[y-1,x+1];
          yield return (p1, p2);
        }

        if(y < rows-1) 
        {
          p2 = meshPoints[y+1,x+1];
          yield return (p1, p2);
        }
      }
    }
  }

  public IEnumerable<(MeshPoint, MeshPoint, MeshPoint)> IterateTriangles()
  {
    for(int i = 0; i < triangles.Length; i+=3)
    {
      MeshPoint p1, p2, p3;
      p1 = triangles[i];
      p2 = triangles[i+1];
      p3 = triangles[i+2];
      yield return (p1, p2, p3);
    }
  }
}