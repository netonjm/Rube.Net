using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Joints;

using System.Text;
using System.IO;
using FarseerPhysics;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Factories;
using FarseerPhysics.Common;

namespace Farseer2Json
{
	public class Nb2dJsonImage
	{
		internal string Name { get; set; }
		internal string File { get; set; }
		internal FarseerPhysics.Dynamics.Body Body { get; set; }
		internal Vector2 Center { get; set; }
		internal float Angle { get; set; }
		internal float Scale { get; set; }
		internal bool Flip { get; set; }
		internal float Opacity { get; set; }
		internal int Filter { get; set; }
		internal float RenderOrder { get; set; }
		internal int[] ColorTint { get; set; }
		internal Vector2[] Corners { get; set; }
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

