using UnityEngine;
using System;
public class Verlet : Integrator
{
  // old
  public void Advance(MeshPoint[,] meshPoints, Action<float> updateForcesFunc, float dt)
  {
    updateForcesFunc(dt);

    for (int i = 0; i < meshPoints.GetLength(0); i++)
    {
      for (int j = 0; j < meshPoints.GetLength(1); j++)
      {
        MeshPoint point = meshPoints[i, j];
        // previous values
        Vector3 prevAcc = point.State.PrevAcceleration;
        Vector3 prevVel = point.State.Velocity;
        Vector3 prevPos = point.State.Position;

        // Set next values
        Vector3 nextAcc = point.Force * 1.0f / point.Mass;
        Vector3 nextPos = prevPos + prevVel * dt + prevAcc * dt * dt / 2;
        Vector3 nextVel = prevVel + (prevAcc + nextAcc) * dt / 2;

        // update point
        point.State.PrevAcceleration = nextAcc;
        point.State.Position = nextPos;
        point.State.Velocity = nextVel;

        // safety belt
        meshPoints[i, j] = point;
      }
    }
  }
  // new
  public void Advance(MeshPointManager mpManager, Action<float> updateForcesFunc, float dt)
  {
    updateForcesFunc(dt);

    foreach (MeshPoint point in mpManager)
    {
      // previous values
      Vector3 prevAcc = point.State.PrevAcceleration;
      Vector3 prevVel = point.State.Velocity;
      Vector3 prevPos = point.State.Position;

      // Set next values
      Vector3 nextAcc = point.Force * 1.0f / point.Mass;
      Vector3 nextPos = prevPos + prevVel * dt + prevAcc * dt * dt / 2;
      Vector3 nextVel = prevVel + (prevAcc + nextAcc) * dt / 2;

      // update point
      point.State.PrevAcceleration = nextAcc;
      point.State.Position = nextPos;
      point.State.Velocity = nextVel;
    }
  }
}
