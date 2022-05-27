using UnityEngine;
public class MeshGenerator
{
  public static Mesh GetMesh(float width, float height, int cols, int rows, Vector3 startPos = default(Vector3))
  {
    Mesh mesh = new Mesh();

    // Create vertices.
    Vector3[] vertices = new Vector3[cols * rows];
    generateVertexArray(vertices, width, height, rows, cols, startPos);
    mesh.vertices = vertices;

    // Create triangles.
    int[] tris = new int[(cols - 1) * (rows - 1) * 2 * 3];
    generateTriangleArray(tris, rows, cols);
    mesh.triangles = tris;
    mesh.RecalculateNormals();

    // Create uv-array.
    Vector2[] uv = new Vector2[cols * rows];
    generateUVArray(uv, cols, rows);
    mesh.uv = uv;

    return mesh;
  }

  // Generate a 2d-grid structure vertexes.
  private static void generateVertexArray(Vector3[] vertices, float width, float height, int rows, int cols, Vector3 startPos)
  {
    float dx = width / (cols - 1);
    float dy = height / (rows - 1);

    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols; x++)
      {

        vertices[y * cols + x] = new Vector3(x * dx, y * dy, 0) + startPos;
      }
    }
  }
 
  private static void generateTriangleArray(int[] tris, int rows, int cols)
  {
    int triangleIdx = 0;
    for (int y = 0; y < rows - 1; y++)
    {
      for (int x = 0; x < cols; x++)
      {
        int vertexIdx = x + cols * y;
        // lower left
        if (x != 0)
        {
          tris[triangleIdx] = vertexIdx - 1;
          tris[triangleIdx + 1] = vertexIdx + cols;
          tris[triangleIdx + 2] = vertexIdx;
          triangleIdx += 3;
        }

        // upper right
        if (x != cols - 1)
        {
          tris[triangleIdx] = vertexIdx;
          tris[triangleIdx + 1] = vertexIdx + cols;
          tris[triangleIdx + 2] = vertexIdx + cols + 1;
          triangleIdx += 3;
        }
      }
    }
  }

  private static void generateUVArray(Vector2[] uv, int rows, int cols)
  {
    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols; x++)
      {
        uv[y * cols + x] = new Vector2((float)x / (cols - 1), (float)y / (rows - 1));
      }
    }
  }
}