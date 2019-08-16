// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Kinect/DepthHistImageShader" {
    Properties {
        //_MainTex ("Base (RGB)", 2D) = "white" {}
		_FrontColor("Front color", Color) = (1, 1, 0, 1)
		_BackColor("Back color", Color) = (0, 0, 1, 1)
		_InvalidColor("Invalid color", Color) = (0, 0, 0, 0)
	}
    
	SubShader {
		Pass {
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }
		
			CGPROGRAM
			#pragma target 5.0
			//#pragma enable_d3d11_debug_symbols

			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"

			//uniform sampler2D _MainTex;
			uniform uint _TexResX;
			uniform uint _TexResY;
			uniform uint _MinDepth;
			uniform uint _MaxDepth;
			uniform uint _TotalPoints;

			uniform float4 _FrontColor;
			uniform float4 _BackColor;
			uniform float4 _InvalidColor;

			StructuredBuffer<uint> _DepthMap;
			StructuredBuffer<uint> _HistMap;

			struct v2f {
				float4 pos : SV_POSITION;
			    float2 uv : TEXCOORD0;
			};

			v2f vert (appdata_base v)
			{
				v2f o;
				
				o.pos = UnityObjectToClipPos (v.vertex);
				o.uv = v.texcoord;
				
				return o;
			}

			float4 frag (v2f i) : COLOR
			{
				uint dx = (uint)(i.uv.x * _TexResX);
				uint dy = (uint)(i.uv.y * _TexResY);
				uint di = (dx + dy * _TexResX);
				
				//return float4((float)dx / (float)_TexResX, (float)dy / (float)_TexResY, 0, 1);

				//int depth = _DepthMap[di];
				bool isOdd = di % 2 == 1;
				uint depth2 = _DepthMap[di >> 1];
				uint depth = (di % 2 == 0 ? depth2 <<= 16 : depth2) >> 16;
				depth = depth >= _MinDepth && depth <= _MaxDepth ? depth : 0;

				float hist = 1.0 - ((float)_HistMap[depth] / (float)_TotalPoints);
				float4 clr = lerp(_BackColor, _FrontColor, hist);

				//return float4(hist, hist, 0, 1); // yellow
				return depth > 0 ? clr : _InvalidColor;
			}

			ENDCG
		}
	}

	Fallback Off
}