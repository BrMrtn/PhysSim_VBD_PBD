using UnityEngine;

[RequireComponent(typeof(XPBDCloth))]
public class XPBDSphereCollision : MonoBehaviour
{
    public SphereCollider sphereCollider;
    public float friction = 0.0f;

    private XPBDCloth clothSim;

    void Start()
    {
        clothSim = GetComponent<XPBDCloth>();
        if (clothSim != null)
        {
            clothSim.OnUpdate += HandleCollision;
        }
    }

    void OnDestroy()
    {
        if (clothSim != null)
            clothSim.OnUpdate -= HandleCollision;
    }

    void HandleCollision()
    {
        if (sphereCollider == null) return;

        Vector3 center = sphereCollider.transform.TransformPoint(sphereCollider.center);
        // Calculate world radius based on scale (assuming uniform scale for simplicity)
        float radius = sphereCollider.radius * sphereCollider.transform.lossyScale.x * 1.1f;

        int numVerts = clothSim.numVerts;
        Vector3[] positions = clothSim.positions;
        Vector3[] prevPositions = clothSim.previousPosition;
        float[] invMasses = clothSim.invMasses;

        for (int i = 0; i < numVerts; i++)
        {
            if (invMasses[i] == 0f) continue;

            Vector3 delta = positions[i] - center;
            float dist = delta.magnitude;

            if (dist < radius)
            {
                Vector3 normal = delta / dist;
                positions[i] = center + normal * radius;

                // Friction logic
                Vector3 displacement = positions[i] - prevPositions[i];
                Vector3 dispNormal = Vector3.Dot(displacement, normal) * normal;
                Vector3 dispTangent = displacement - dispNormal;

                // Apply friction
                positions[i] -= dispTangent * friction;
            }
        }
    }
}