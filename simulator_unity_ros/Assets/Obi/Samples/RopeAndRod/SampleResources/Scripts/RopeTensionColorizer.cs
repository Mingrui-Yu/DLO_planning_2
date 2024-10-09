using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

[RequireComponent(typeof(ObiRope))]
[RequireComponent(typeof(MeshRenderer))]
public class RopeTensionColorizer : MonoBehaviour
{
    public float minTension = 0;
    public float maxTension = 0.2f;
    public Color normalColor = Color.green;
	public Color tensionColor = Color.red;

    public RopeTenser tenser;
    public float tenserThreshold = -5;
    public float tenserMax = 0.1f;

    private ObiRope rope;
    private Material localMaterial;

    void Awake()
    {
		rope = GetComponent<ObiRope>();
        localMaterial = GetComponent<MeshRenderer>().material;
	}

    private void OnDestroy()
    {
        Destroy(localMaterial);
    }

    void Update()
    {
        if (tenser == null)
            return;

        // Calculate how much past the threshold the tenser is, clamp the excess to 1
        float tenserFactor = Mathf.Min((tenser.transform.position.y - tenserThreshold) / tenserMax,1);

        // Check if the tenser is above the threshold, if so then check rope tension:
        if (tenserFactor > 0)
        {
            // Calculate current tension as ratio between current and rest length, then subtract 1 to center it at zero.
            float tension = rope.CalculateLength() / rope.restLength - 1;

            // Use tension to interpolate between colors:
            float lerpFactor = (tension - minTension) / (maxTension - minTension);
            localMaterial.color = Color.Lerp(normalColor, tensionColor, lerpFactor * tenserFactor);
        }
        else
        {
            localMaterial.color = normalColor;
        }
    }
}
