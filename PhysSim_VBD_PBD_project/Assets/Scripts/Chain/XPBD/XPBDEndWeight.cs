using UnityEngine;

[RequireComponent(typeof(XPBDChain))]
public class XPBDEndWeight : MonoBehaviour
{
    [Tooltip("Mass of the last particle in the chain. Set to 0 to fix the particle in place.")]
    public float mass = 1f;

    private XPBDChain chain;

    void Start()
    {
        chain = GetComponent<XPBDChain>();
        if (chain == null) return;

        int idx = chain.Solver.numVerts - 1;
        chain.Solver.invMasses[idx] = mass <= 0f ? 0f : 1f / mass;
    }
}
