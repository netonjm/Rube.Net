using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Newtonsoft.Json.Linq;
using System.Text;
using System.IO;
using Box2D.Dynamics;
using Box2D.Common;


namespace Farseer2Json
{
	public class Nb2dJsonImage
	{
		internal string Name { get; set; }
		internal string File { get; set; }
        internal b2Body Body { get; set; }
		internal b2Vec2 Center { get; set; }
		internal float Angle { get; set; }
		internal float Scale { get; set; }
		internal bool Flip { get; set; }
		internal float Opacity { get; set; }
		internal int Filter { get; set; }
		internal float RenderOrder { get; set; }
		internal int[] ColorTint { get; set; }
        internal b2Vec2[] Corners { get; set; }
		internal int NumPoints { get; set; }
		internal float[] Points { get; set; }
		internal float[] UvCoords { get; set; }
		internal int NumIndices { get; set; }
		internal short[] Indices { get; set; }
		
		
		public Nb2dJsonImage()
		{
			ColorTint = new int[4];
		}
		
	}
}

