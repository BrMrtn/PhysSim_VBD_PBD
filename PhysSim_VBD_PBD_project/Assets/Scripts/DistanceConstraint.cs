using UnityEngine;

public class DistanceConstraint
{
    public ClothParticle p1;
    public ClothParticle p2;
    public float restLength;

    public DistanceConstraint(ClothParticle a, ClothParticle b, float rL)
    {
        p1 = a;
        p2 = b;
        restLength = rL;
    }

    public void Solve()
    {
        Vector3 delta = p2.position - p1.position;
        float dist = delta.magnitude;
        float diff = (dist - restLength) / dist;
    }
}