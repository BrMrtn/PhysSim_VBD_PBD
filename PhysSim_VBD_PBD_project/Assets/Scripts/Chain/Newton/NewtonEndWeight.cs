using UnityEngine;

[RequireComponent(typeof(NewtonChain))]
public class NewtonEndWeight : MonoBehaviour
{
    [Tooltip("Mass of the last particle in the chain. Set to 0 to fix the particle in place.")]
    public float mass = 1f;

    private NewtonChain chain;

    void Start()
    {
        chain = GetComponent<NewtonChain>();
        if (chain == null) return;

        int idx = chain.Solver.numVerts - 1;
        chain.Solver.masses[idx] = mass <= 0f ? 1f : mass;
        chain.Solver.invMasses[idx] = mass <= 0f ? 0f : 1f / mass;
    }
}
