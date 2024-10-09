using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Obi;

public class ExtrapolationCamera : MonoBehaviour
{
    public Transform target = null;

    public float extrapolation = 10;

    [Range(0, 1)]
    public float smoothness = 0.8f;

    [Range(0, 1)]
    public float linearSpeed = 1;

    [Range(0, 1)]
    public float rotationalSpeed = 1;

    [Min(0)]
    public float distanceFromTarget = 4;

    Vector3 lastPosition;
    Vector3 extrapolatedPos;

    void Start()
    {
        if (target != null)
            lastPosition = target.position;
    }

    private void FixedUpdate()
    {
        if (target != null)
        {
			// Get position delta since the last physics update:
			Vector3 positionDelta = target.position - lastPosition;
            positionDelta.y = 0;

            // extrapolate position using velocity (the division/multiplication by Time.deltaTime simplify out)
            extrapolatedPos = Vector3.Lerp(target.position + positionDelta * extrapolation, extrapolatedPos, smoothness);

            // store the target's current com for the next frame:
            lastPosition = target.position;
        }
    }

    void LateUpdate()
    {
        if (target != null)
        {
            // get vector from the camera to the extrapolated position:
            Vector3 toTarget = extrapolatedPos - transform.position;

            // rotate the camera towards the extrapolated position:
            transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(toTarget), rotationalSpeed);

            // keep our current world space height:
            toTarget.y = 0;

            // move the camera towards the extrapolated position, keeping some distance to it:
            transform.position += toTarget.normalized * (toTarget.magnitude - distanceFromTarget) * linearSpeed;

        }
    }

    public void Teleport(Vector3 position, Quaternion rotation)
    {
        transform.position = position;
        transform.rotation = rotation;

        if (target != null)
			extrapolatedPos = lastPosition = target.position;
    }
}
