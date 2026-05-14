using UnityEngine;

[RequireComponent(typeof(XPBDChain))]
public class XPBDFixedEnd : MonoBehaviour
{

    private XPBDChain chain;

    void Start()
    {
        chain = GetComponent<XPBDChain>();
        if (chain == null) return;

        chain.Solver.invMasses[0] = 0f;
    }
}
