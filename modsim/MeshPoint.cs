using UnityEngine;
using System.Collections;

public class MeshPoint
{
    public class PointState
    {
        public PointState()
        {
            Position = Vector3.zero;
            Velocity = Vector3.zero;
        }

        public PointState(Vector3 pos, Vector3 vel)
        {
            Position = pos;
            Velocity = vel;
        }

        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 PrevAcceleration;

        public PointState Clone()
        {
            return new PointState(Position, Velocity);
        }
    }

    public MeshPoint(Vector3 pos, Vector3 vel = default(Vector3))
    {
      State = new PointState(pos, vel);
      PrevState = new PointState(pos, vel);
    }

    public PointState State { get; set; }
    public PointState PrevState { get; set; }

   
    public Vector3 Force{ get; private set; }
    public float Mass = 1.0f;

    public void ClearForce()
    {
        Force = Vector3.zero;
    }

    public void ClearSpeed()
    {
        State.Velocity = Vector3.zero;
    }

    public void AddForce(Vector3 force)
    {
        Force += force;
    }

    public void SaveState()
    {
        m_savedState = State.Clone();
    }
    public void LoadState()
    {
        State = m_savedState.Clone();
    }

    // Helpful when using RK4 integration 
    private PointState m_savedState = new PointState();
}
