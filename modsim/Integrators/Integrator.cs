using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

public enum IntegratorType
{ 
    Euler,
    Leapfrog,
    RK4,
    Verlet
}

public interface Integrator
{
    void Advance(MeshPointManager points, Action<float> updateForcesFunc, float timeStep);
}
