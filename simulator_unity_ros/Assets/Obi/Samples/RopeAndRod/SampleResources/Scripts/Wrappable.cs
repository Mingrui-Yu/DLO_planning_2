using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wrappable : MonoBehaviour
{

	private bool wrapped = false;
	public Color normalColor = new Color(0.2f,0.2f,0.8f);
	public Color wrappedColor = new Color(0.9f, 0.9f, 0.2f);

	Material localMaterial;

    public void Awake()
	{
		localMaterial = GetComponent<MeshRenderer>().material;
	}

    public void OnDestroy()
	{
		Destroy(localMaterial);
	}

	public void Reset()
	{
		wrapped = false;
		localMaterial.color = normalColor;
	}

    public void SetWrapped()
	{
		wrapped = true;
		localMaterial.color = wrappedColor;
	}

	public bool IsWrapped()
	{
		return wrapped;
	}

}
