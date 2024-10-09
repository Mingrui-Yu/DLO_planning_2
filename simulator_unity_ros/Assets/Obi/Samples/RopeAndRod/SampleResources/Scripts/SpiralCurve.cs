using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

[ExecuteInEditMode]
public class SpiralCurve : MonoBehaviour {

	public float radius = 0.25f;
	public float radialStep = 0.8f;
	public float heightStep = 0.04f;
	public float points = 30;

    public float rotationalMass = 1;
    public float thickness = 1;

    void Awake ()
    {
        Generate();
    }

    public void Generate()
    {
        var rod = GetComponent<ObiRopeBase>();
        if (rod == null) return;

        var blueprint = rod.sourceBlueprint as ObiRopeBlueprintBase;
        if (blueprint == null) return;

        blueprint.path.Clear();

        float ang = 0;
        float height = 0;

        for (int i = 0; i < points; ++i)
        {
            Vector3 point = new Vector3(Mathf.Cos(ang) * radius, height, Mathf.Sin(ang) * radius);

            // optimal handle length for circle approximation: 4/3 tan(pi/(2n))
            Vector3 tangent = new Vector3(-point.z, heightStep, point.x).normalized * (4.0f / 3.0f) * Mathf.Tan(radialStep / 4.0f) * radius;

            blueprint.path.AddControlPoint(point, -tangent, tangent, Vector3.up, 1, rotationalMass, thickness, 1, Color.white, "control point " + i);
            ang += radialStep;
            height += heightStep;
        }

        blueprint.path.FlushEvents();
    }

}
