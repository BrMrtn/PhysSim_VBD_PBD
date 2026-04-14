using UnityEngine;

public class ClothParticle
{
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 predictedPosition;
    public float invMass = 1f;

    public ClothParticle(Vector3 position, float invMass)
    {
        this.position = position;
        this.velocity = Vector3.zero;
        this.predictedPosition = position;
        this.invMass = invMass;
    }

    public void ApplyForce(Vector3 force, float dt)
    {
        velocity += force * invMass * dt;
        // *damp velocities*
        predictedPosition = position + velocity * dt;
        // generate constraints (with neighbour vertices)
        // project contraints (solverIteration times) - this changes predicted position
        velocity = (predictedPosition - position) / dt;
        position = predictedPosition;
    }
}