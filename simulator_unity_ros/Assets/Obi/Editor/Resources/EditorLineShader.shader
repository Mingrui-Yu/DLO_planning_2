Shader "Obi/EditorLines" 
{
	SubShader 
	{ 
		Pass 
		{
		    Blend SrcAlpha OneMinusSrcAlpha 
			Cull Off 
			ZTest Always
			Fog { Mode Off }  
		    BindChannels 
			{
		      Bind "vertex", vertex Bind "color", color 
			}
		} 
	} 
}