using UnityEngine;

public class Blinker : MonoBehaviour {

	public Color highlightColor;

 	private Renderer rend;
	private Color original;

	void Awake(){
		rend = GetComponent<Renderer>();
		original = rend.material.color;
	}

	public void Blink(){
		rend.material.color = highlightColor;
	}

	void LateUpdate(){
		rend.material.color += (original - rend.material.color)*Time.deltaTime*5;
	}

}
