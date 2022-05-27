using System;
public class EulerIntegrator : Integrator
{
    public void Advance(MeshPointManager points, Action<float> updateForcesFunc, float timeStep)
  {
    updateForcesFunc(timeStep);

    foreach (MeshPoint point in points)
    {
      // v(k+1) = dy/dt = v0 + at
      point.State.Velocity += (timeStep / point.Mass) * point.Force;
      // y(k+1) = y(k) + v(k+1)*h
      point.State.Position += timeStep * point.State.Velocity;
    }
  }
}
