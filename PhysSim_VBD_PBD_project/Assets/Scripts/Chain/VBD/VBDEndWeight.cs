using UnityEngine;

[RequireComponent(typeof(VBDChain))]
public class VBDEndWeight : MonoBehaviour
{
    [Tooltip("Mass of the last particle in the chain. Set to 0 to fix the particle in place.")]
    public float mass = 1f;

    [Tooltip("Render size multiplier for the last node's sphere (1 = same size as the other nodes).")]
    public float bobSize = 1f;

    private VBDChain chain;

    void Start()
    {
        chain = GetComponent<VBDChain>();
        if (chain == null) return;

        int idx = chain.Solver.numVerts - 1;
        chain.Solver.masses[idx] = mass <= 0f ? 1f : mass;
        chain.Solver.invMasses[idx] = mass <= 0f ? 0f : 1f / mass;
        chain.SetVertexSphereScale(idx, bobSize);
    }
}
