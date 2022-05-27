using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

public class LeapfrogIntegrator : Integrator
{
    private int step = 0;
    public void Advance(MeshPointManager points, Action<float> updateForcesFunc, float timeStep)
    {
        updateForcesFunc(timeStep);
        if (step % 2 == 0) 
        {
            foreach (MeshPoint point in points)
            {
                point.State.Position += timeStep * point.State.Velocity;

            }
        } 
        else 
        {
            foreach (MeshPoint point in points)
            {
                point.State.Velocity += (timeStep / point.Mass) * point.Force;
            }
        }

        step = ++step % 2;
    }
}