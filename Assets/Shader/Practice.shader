//----参考にしたページ
//----http://nn-hokuson.hatenablog.com/entry/2017/03/27/204255

Shader "Custom/Practice" {
	Properties {
		_Color ("Color", Color) = (1,1,1,1)
		_MainTex ("Albedo (RGB)", 2D) = "white" {}
		_RampTex ("Ramp", 2D) = "White" {} //★追加
		//----_Glossiness ("Smoothness", Range(0,1)) = 0.5
		//----_Metallic ("Metallic", Range(0,1)) = 0.0
	}
	SubShader {
		Tags { "RenderType"="Opaque" }
		LOD 200
		
		CGPROGRAM
		// Physically based Standard lighting model, and enable shadows on all light types
		//----#pragma surface surf Standard fullforwardshadows
		#pragma surface surf ToonRamp //★

		// Use shader model 3.0 target, to get nicer looking lighting
		#pragma target 3.0

		sampler2D _MainTex;
		sampler2D _RampTex; //★

		struct Input {
			float2 uv_MainTex;
		};

		//----half _Glossiness;
		//----half _Metallic;
		fixed4 _Color;

		//★以下の関数1つ追加
		fixed4 LightingToonRamp (SurfaceOutput s, fixed3 lightDir, fixed atten)
		{
			half d = dot(s.Normal, lightDir)*0.5 + 0.5;
			fixed3 ramp = tex2D(_RampTex, fixed2(d, 0.5)).rgb;
			fixed4 c;
			c.rgb = s.Albedo * _LightColor0.rgb * ramp;
			c.a = 0;
			return c;
		}

		// Add instancing support for this shader. You need to check 'Enable Instancing' on materials that use the shader.
		// See https://docs.unity3d.com/Manual/GPUInstancing.html for more information about instancing.
		// #pragma instancing_options assumeuniformscaling
		//----UNITY_INSTANCING_CBUFFER_START(Props)
			// put more per-instance properties here
		//----UNITY_INSTANCING_CBUFFER_END

		void surf (Input IN, inout SurfaceOutput o) {
			// Albedo comes from a texture tinted by color
			fixed4 c = tex2D (_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = c.rgb;
			// Metallic and smoothness come from slider variables
			//----o.Metallic = _Metallic;
			//----o.Smoothness = _Glossiness;
			o.Alpha = c.a;
		}
		ENDCG
	}
	FallBack "Diffuse"
}
