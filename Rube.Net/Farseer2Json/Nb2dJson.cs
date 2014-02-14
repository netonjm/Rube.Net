using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Joints;
using Newtonsoft.Json.Linq;
using System.Text;
using System.IO;
using FarseerPhysics;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Factories;
using FarseerPhysics.Common;
using Farseer2Json;

namespace Farseer2Json
{
	
	
	public class Nb2dJson
	{
		public class Nb2dJsonCustomProperties
		{
			public Dictionary<string, int> m_customPropertyMap_int = new Dictionary<string, int>();
			public Dictionary<string, double> m_customPropertyMap_float = new Dictionary<string, double>();
			public Dictionary<string, string> m_customPropertyMap_string = new Dictionary<string, string>();
			public Dictionary<string, Vector2> m_customPropertyMap_vec2 = new Dictionary<string, Vector2>();
			public Dictionary<string, bool> m_customPropertyMap_bool = new Dictionary<string, bool>();
		}
		
		protected bool m_useHumanReadableFloats;
		protected int m_simulationPositionIterations;
		protected int m_simulationVelocityIterations;
		protected float m_simulationFPS;
		
		protected Dictionary<int, Body> m_indexToBodyMap;
		protected Dictionary<Body, int?> m_bodyToIndexMap;
		protected Dictionary<Joint, int?> m_jointToIndexMap;
		protected List<Body> m_bodies;
		protected List<Joint> m_joints;
		protected List<Nb2dJsonImage> m_images;
		
		protected Dictionary<Body, string> m_bodyToNameMap;
		protected Dictionary<Fixture, string> m_fixtureToNameMap;
		protected Dictionary<Joint, string> m_jointToNameMap;
		protected Dictionary<Nb2dJsonImage, string> m_imageToNameMap;
		
		
		// This maps an item (Body, Fixture etc) to a set of custom properties.
		// Use null for world properties.
		protected Dictionary<object, Nb2dJsonCustomProperties> m_customPropertiesMap;
		
		protected HashSet<Body> m_bodiesWithCustomProperties;
		protected HashSet<Fixture> m_fixturesWithCustomProperties;
		protected HashSet<Joint> m_jointsWithCustomProperties;
		protected HashSet<Nb2dJsonImage> m_imagesWithCustomProperties;
		protected HashSet<World> m_worldsWithCustomProperties;
		
		
		public Nb2dJson()
			: this(true)
		{
			
		}
		
		public Nb2dJson(bool useHumanReadableFloats)
		{
			if (!useHumanReadableFloats)
			{
				// The floatToHex function is not giving the same results
				// as the original C++ version... not critical so worry about it
				// later.
				Console.WriteLine("Non human readable floats are not implemented yet");
				useHumanReadableFloats = true;
			}
			
			m_useHumanReadableFloats = useHumanReadableFloats;
			m_simulationPositionIterations = 3;
			m_simulationVelocityIterations = 8;
			m_simulationFPS = 60;
			
			m_indexToBodyMap = new Dictionary<int, Body>();
			m_bodyToIndexMap = new Dictionary<Body, int?>();
			m_jointToIndexMap = new Dictionary<Joint, int?>();
			m_bodies = new List<Body>();
			m_joints = new List<Joint>();
			m_images = new List<Nb2dJsonImage>();
			
			m_bodyToNameMap = new Dictionary<Body, string>();
			m_fixtureToNameMap = new Dictionary<Fixture, string>();
			m_jointToNameMap = new Dictionary<Joint, string>();
			m_imageToNameMap = new Dictionary<Nb2dJsonImage, string>();
			
			m_customPropertiesMap = new Dictionary<object, Nb2dJsonCustomProperties>();
			
			m_bodiesWithCustomProperties = new HashSet<Body>();
			m_fixturesWithCustomProperties = new HashSet<Fixture>();
			m_jointsWithCustomProperties = new HashSet<Joint>();
			m_imagesWithCustomProperties = new HashSet<Nb2dJsonImage>();
			m_worldsWithCustomProperties = new HashSet<World>();
		}
		
		public bool WriteToFile(World world, String filename, int indentFactor, StringBuilder errorMsg)
		{
			if (null == world || null == filename)
				return false;
			
			
			using (System.IO.TextWriter writeFile = new StreamWriter(filename))
			{
				try
				{
					writeFile.WriteLine(B2n(world).ToString());
				}
				catch (Exception e)
				{
					errorMsg.Append("Error writing JSON to file: " + filename + "  " + e.Message);
					return false;
				}
			}
			
			return true;
		}
		
		public JObject B2n(World world)
		{
			JObject worldValue = new JObject();
			
			m_bodyToIndexMap.Clear();
			m_jointToIndexMap.Clear();
			
			VecToJson("gravity", world.Gravity, worldValue);
			worldValue["allowSleep"] = Settings.AllowSleep;
			worldValue["autoClearForces"] = world.AutoClearForces;
			worldValue["warmStarting"] = Settings.EnableWarmstarting;
			worldValue["continuousPhysics"] = Settings.ContinuousPhysics;
			
			JArray customPropertyValue = WriteCustomPropertiesToJson(null);
			if (customPropertyValue.Count > 0)
				worldValue["customProperties"] = customPropertyValue;
			
			int i = 0;
			JArray arr = new JArray(); ;
			foreach (var item in world.BodyList)
			{
				m_bodyToIndexMap.Add(item, i);
				arr.Add(B2n(item));
				i++;
			}
			worldValue["body"] = arr;
			
			arr = new JArray();
			// need two passes for joints because gear joints reference other joints
			foreach (var joint in world.JointList)
			{
				if (joint.JointType == JointType.Gear)
					continue;
				m_jointToIndexMap[joint] = i;
				arr.Add( B2n(joint) );
				i++;
			}
			
			foreach (var joint in world.JointList)
			{
				if (joint.JointType != JointType.Gear)
					continue;
				m_jointToIndexMap[joint] = i;
				arr.Add(B2n(joint));
				i++;
			}
			worldValue["joint"] = arr;
			
			
			arr = new JArray();
			// Currently the only efficient way to add images to a Jb2dJson
			// is by using the R.U.B.E editor. This code has not been tested,
			// but should work ok.
			foreach (var image in m_imageToNameMap.Keys)
			{
				arr.Add(B2n(image));
			}
			worldValue["image"] = arr;
			
			
			m_bodyToIndexMap.Clear();
			m_jointToIndexMap.Clear();
			return worldValue;
		}
		
		JObject B2n(Nb2dJsonImage image)
		{
			JObject imageValue = new JObject();
			
			if (null != image.Body)
				imageValue["body"] = lookupBodyIndex(image.Body);
			else
				imageValue["body"] = -1;
			
			if (null != image.Name)
				imageValue["name"] = image.Name;
			if (null != image.File)
				imageValue["file"] = image.File;
			
			VecToJson("center", image.Center, imageValue);
			FloatToJson("angle", image.Angle, imageValue);
			FloatToJson("scale", image.Scale, imageValue);
			if (image.Flip)
				imageValue["flip"] = true;
			FloatToJson("opacity", image.Opacity, imageValue);
			imageValue["filter"] = image.Filter;
			FloatToJson("renderOrder", image.RenderOrder, imageValue);
			
			bool defaultColorTint = true;
			for (int i = 0; i < 4; i++)
			{
				if (image.ColorTint[i] != 255)
				{
					defaultColorTint = false;
					break;
				}
			}
			
			if (!defaultColorTint)
			{
				JArray array = (JArray)imageValue["colorTint"];
				for (int i = 0; i < 4; i++)
					array[i] = image.ColorTint[i];
			}
			
			// image->updateCorners();
			for (int i = 0; i < 4; i++)
				VecToJson("corners", image.Corners[i], imageValue, i);
			
			// image->updateUVs();
			for (int i = 0; i < 2 * image.NumPoints; i++)
			{
				VecToJson("glVertexPointer", image.Points[i], imageValue, i);
				VecToJson("glTexCoordPointer", image.UvCoords[i], imageValue, i);
			}
			for (int i = 0; i < image.NumIndices; i++)
				VecToJson("glDrawElements", image.Indices[i], imageValue, i);
			
			JArray customPropertyValue = WriteCustomPropertiesToJson(image);
			if (customPropertyValue.Count > 0)
				imageValue["customProperties"] = customPropertyValue;
			
			return imageValue;
		}
		
		public JObject B2n(Joint joint)
		{
			JObject jointValue = new JObject();
			
			String jointName = GetJointName(joint);
			if (null != jointName)
				jointValue["name"] = jointName;
			
			int bodyIndexA = lookupBodyIndex(joint.BodyA);
			int bodyIndexB = lookupBodyIndex(joint.BodyB);
			jointValue["bodyA"] = bodyIndexA;
			jointValue["bodyB"] = bodyIndexB;
			if (joint.CollideConnected)
				jointValue["collideConnected"] = true;
			
			Body bodyA = joint.BodyA;
			Body bodyB = joint.BodyB;
			
			// why do Joint.getAnchor methods need to take an argOut style
			// parameter!?
			Vector2 tmpAnchor = new Vector2();
			
			switch (joint.JointType)
			{
			case JointType.Revolute:
			{
				jointValue["type"] = "revolute";
				
				RevoluteJoint revoluteJoint = (RevoluteJoint)joint;
				tmpAnchor = revoluteJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = revoluteJoint.WorldAnchorA;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				
				
				FloatToJson("refAngle", bodyB.Rotation - bodyA.Rotation - revoluteJoint.JointAngle, jointValue);
				FloatToJson("jointSpeed", revoluteJoint.JointSpeed, jointValue);
				jointValue["enableLimit"] = revoluteJoint.LimitEnabled;
				FloatToJson("lowerLimit", revoluteJoint.LowerLimit, jointValue);
				FloatToJson("upperLimit", revoluteJoint.UpperLimit, jointValue);
				jointValue["enableMotor"] = revoluteJoint.Enabled;
				FloatToJson("motorSpeed", revoluteJoint.MotorSpeed, jointValue);
				FloatToJson("maxMotorTorque", revoluteJoint.MaxMotorTorque, jointValue);
			}
				break;
			case JointType.Prismatic:
			{
				jointValue["type"] = "prismatic";
				
				PrismaticJoint prismaticJoint = (PrismaticJoint)joint;
				tmpAnchor = prismaticJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = prismaticJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				VecToJson("localAxisA", prismaticJoint.LocalXAxisA, jointValue);
				FloatToJson("refAngle", prismaticJoint.ReferenceAngle, jointValue);
				jointValue["enableLimit"] = prismaticJoint.LimitEnabled;
				FloatToJson("lowerLimit", prismaticJoint.LowerLimit, jointValue);
				FloatToJson("upperLimit", prismaticJoint.UpperLimit, jointValue);
				jointValue["enableMotor"] = prismaticJoint.MotorEnabled;
				FloatToJson("maxMotorForce", prismaticJoint.MaxMotorForce, jointValue);
				FloatToJson("motorSpeed", prismaticJoint.MotorSpeed, jointValue);
			}
				break;
			case JointType.Distance:
			{
				jointValue["type"] = "distance";
				
				DistanceJoint distanceJoint = (DistanceJoint)joint;
				tmpAnchor = distanceJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = distanceJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("length", distanceJoint.Length, jointValue);
				FloatToJson("frequency", distanceJoint.Frequency, jointValue);
				FloatToJson("dampingRatio", distanceJoint.DampingRatio, jointValue);
			}
				break;
			case JointType.Pulley:
			{
				jointValue["type"] = "pulley";
				
				PulleyJoint pulleyJoint = (PulleyJoint)joint;
				VecToJson("groundAnchorA", pulleyJoint.GroundAnchorA, jointValue);
				VecToJson("groundAnchorB", pulleyJoint.GroundAnchorB, jointValue);
				tmpAnchor = pulleyJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("lengthA", (pulleyJoint.GroundAnchorA - tmpAnchor).Length(), jointValue);
				tmpAnchor = pulleyJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("lengthB", (pulleyJoint.GroundAnchorB - tmpAnchor).Length(), jointValue);
				FloatToJson("ratio", pulleyJoint.Ratio, jointValue);
			}
				break;
			case JointType.FixedMouse:
			{
				jointValue["type"] = "mouse";
				
				FixedMouseJoint mouseJoint = (FixedMouseJoint)joint;
				VecToJson("target", mouseJoint.WorldAnchorB, jointValue);
				tmpAnchor = mouseJoint.WorldAnchorB;
				VecToJson("anchorB", tmpAnchor, jointValue);
				FloatToJson("maxForce", mouseJoint.MaxForce, jointValue);
				FloatToJson("frequency", mouseJoint.Frequency, jointValue);
				FloatToJson("dampingRatio", mouseJoint.DampingRatio, jointValue);
			}
				break;
			case JointType.Gear:
			{
				jointValue["type"] = "gear";
				GearJoint gearJoint = (GearJoint)joint;
				int jointIndex1 = lookupJointIndex(gearJoint.Joint1);
				int jointIndex2 = lookupJointIndex(gearJoint.Joint2);
				jointValue["joint1"] = jointIndex1;
				jointValue["joint2"] = jointIndex2;
				jointValue["ratio"] = gearJoint.Ratio;
			}
				break;
			case JointType.Wheel:
			{
				jointValue["type"] =  "wheel";
				WheelJoint wheelJoint = (WheelJoint)joint;
				tmpAnchor = wheelJoint.LocalAnchorA;
				VecToJson("anchorA", tmpAnchor, jointValue);
				tmpAnchor = wheelJoint.LocalAnchorB;
				VecToJson("anchorB", tmpAnchor, jointValue);
				VecToJson("localAxisA", wheelJoint.LocalXAxisA  , jointValue);
				jointValue["enableMotor"] = wheelJoint.MotorEnabled;
				FloatToJson("motorSpeed", wheelJoint.MotorSpeed, jointValue);
				FloatToJson("maxMotorTorque", wheelJoint.MaxMotorTorque, jointValue);
				FloatToJson("springFrequency", wheelJoint.SpringFrequencyHz, jointValue);
				FloatToJson("springDampingRatio", wheelJoint.SpringDampingRatio, jointValue);
				
				
			}
				break;
			case JointType.Weld:
			{
				jointValue["type"] = "weld";
				
				WeldJoint weldJoint = (WeldJoint)joint;
				tmpAnchor = weldJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = weldJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("refAngle", weldJoint.ReferenceAngle, jointValue);
                FloatToJson("frequency", weldJoint.FrequencyHz, jointValue);
                FloatToJson("dampingRatio", weldJoint.DampingRatio, jointValue);
            }
				break;
			case JointType.Friction:
			{
				jointValue["type"] = "friction";
				
				FrictionJoint frictionJoint = (FrictionJoint)joint;
				tmpAnchor = frictionJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = frictionJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("maxForce", frictionJoint.MaxForce, jointValue);
				FloatToJson("maxTorque", frictionJoint.MaxTorque, jointValue);
			}
				break;
			case JointType.Rope:
			{
				// Rope joints are apparently not implemented in JBox2D yet, but
				// when they are, commenting out the following section should work.
				
				jointValue["type"] = "rope";
				
				RopeJoint ropeJoint = (RopeJoint)joint;
				tmpAnchor = ropeJoint.WorldAnchorA;
				VecToJson("anchorA", bodyA.GetLocalPoint(tmpAnchor), jointValue);
				tmpAnchor = ropeJoint.WorldAnchorB;
				VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("maxLength", ropeJoint.MaxLength, jointValue);
				
			}
				break;
				
				
			case JointType.Motor:
			{
				jointValue["type"] = "motor";
				MotorJoint motor = (MotorJoint)joint;
				tmpAnchor = motor.WorldAnchorA;
				VecToJson("anchorA", motor.LinearOffset, jointValue);
				//tmpAnchor = motor.WorldAnchorB;
				//VecToJson("anchorB", bodyB.GetLocalPoint(tmpAnchor), jointValue);
				FloatToJson("maxForce", motor.MaxForce, jointValue);
				FloatToJson("maxTorque", motor.MaxTorque, jointValue);
				FloatToJson("refAngle", motor.AngularOffset, jointValue);
			}
				break;
				
				
				
				
				
				
			default:
				Console.WriteLine("Unknown joint type : " + joint.JointType);
				break;
			}
			
			JArray customPropertyValue = WriteCustomPropertiesToJson(joint);
			if (customPropertyValue.Count > 0)
				jointValue["customProperties"] = customPropertyValue;
			
			return jointValue;
		}
		
		public JObject B2n(Body body)
		{
			JObject bodyValue = new JObject();
			
			string bodyName = GetBodyName(body);
			if (null != bodyName)
				bodyValue["name"] = bodyName;
			
			switch (body.BodyType)
			{
			case BodyType.Static:
				bodyValue["type"] = 0;
				break;
			case BodyType.Kinematic:
				bodyValue["type"] = 1;
				break;
			case BodyType.Dynamic:
				bodyValue["type"] = 2;
				break;
			}
			
			VecToJson("position", body.Position, bodyValue);
			FloatToJson("angle", body.Rotation, bodyValue);
			
			VecToJson("linearVelocity", body.LinearVelocity, bodyValue);
			FloatToJson("angularVelocity", body.AngularVelocity, bodyValue);
			
			if (body.LinearDamping != 0)
				FloatToJson("linearDamping", body.LinearDamping, bodyValue);
			if (body.AngularDamping != 0)
				FloatToJson("angularDamping", body.AngularDamping, bodyValue);
			if (body.GravityScale != 1)
				FloatToJson("gravityScale", body.GravityScale, bodyValue);
			
			if (body.IsBullet)
				bodyValue["bullet"] = true;
			if (!body.SleepingAllowed)
				bodyValue["allowSleep"] = false;
			if (body.Awake)
				bodyValue["awake"] = true;
			if (!body.Enabled)
				bodyValue["active"] = false;
			if (body.FixedRotation)
				bodyValue["fixedRotation"] = true;
			
			
			
			/*MassData massData = new MassData();
            massData.Mass = body.Mass;
            massData.Centroid = body.LocalCenter; //im not sure
            massData.Inertia = body.Inertia;*/
			//body.getMassData(massData);
			if (body.Mass != 0)
				FloatToJson("massData-mass", body.Mass, bodyValue);
			if (body.LocalCenter.X != 0 || body.LocalCenter.Y != 0)
				VecToJson("massData-center", body.LocalCenter, bodyValue);
			if (body.Inertia != 0)
			{
				FloatToJson("massData-I", body.Inertia, bodyValue);
			}
			
			
			JArray arr = new JArray();
			foreach (var fixture in body.FixtureList)
			{
				arr.Add( B2n(fixture) );
			}
			bodyValue["fixture"] = arr;
			
			JArray customPropertyValue = WriteCustomPropertiesToJson(body);
			if (customPropertyValue.Count > 0)
				bodyValue["customProperties"] = customPropertyValue;
			
			return bodyValue;
		}
		
		public JObject B2n(Fixture fixture)
		{
			JObject fixtureValue = new JObject();
			
			
			
			String fixtureName = GetFixtureName(fixture);
			if (null != fixtureName)
				fixtureValue["name"] = fixtureName;
			
			if (fixture.Restitution != 0)
				FloatToJson("restitution", fixture.Restitution, fixtureValue);
			if (fixture.Friction != 0)
				FloatToJson("friction", fixture.Friction, fixtureValue);
			if (fixture.Shape.Density != 0)
				FloatToJson("density", fixture.Shape.Density, fixtureValue);
			if (fixture.IsSensor)
				fixtureValue["sensor"] = true;
			
			
			
			
			//is this right??? we need check!!
			if ((int)fixture.CollisionCategories != 0x0001)
				fixtureValue["filter-categoryBits"] = (int)fixture.CollisionCategories;
			if ((int)fixture.CollidesWith != 0xffff)
				fixtureValue["filter-maskBits"] = (int)fixture.CollidesWith;
			if (fixture.CollisionGroup != 0)
				fixtureValue["filter-groupIndex"] = fixture.CollisionGroup;
			
			
			
			Shape shape = fixture.Shape;
			switch (shape.ShapeType)
			{
			case ShapeType.Circle:
			{
				CircleShape circle = (CircleShape)shape;
				JObject shapeValue = new JObject();
				FloatToJson("radius", circle.Radius, shapeValue);
				VecToJson("center", circle.Position, shapeValue);
				fixtureValue["circle"] = shapeValue;
			}
				break;
			case ShapeType.Edge:
			{
				EdgeShape edge = (EdgeShape)shape;
				JObject shapeValue = new JObject();
				VecToJson("vertex1", edge.Vertex1, shapeValue);
				VecToJson("vertex2", edge.Vertex2, shapeValue);
				if (edge.HasVertex0)
					shapeValue["hasVertex0"] = true;
				if (edge.HasVertex3)
					shapeValue["hasVertex3"] = true;
				if (edge.HasVertex0)
					VecToJson("vertex0", edge.Vertex0, shapeValue);
				if (edge.HasVertex3)
					VecToJson("vertex3", edge.Vertex3, shapeValue);
				fixtureValue["edge"] = shapeValue;
			}
				break;
			case ShapeType.Chain:
			{
				ChainShape chain = (ChainShape)shape;
				JObject shapeValue = new JObject();
				int count = chain.Vertices.Count;
				for (int i = 0; i < count; ++i)
					VecToJson("vertices", chain.Vertices[i], shapeValue, i);
				if (chain.PrevVertex != null && chain.PrevVertex != Vector2.Zero)
				{
					shapeValue["hasPrevVertex"] = true;
					VecToJson("prevVertex", chain.PrevVertex, shapeValue);
				}
				if (chain.NextVertex != null && chain.NextVertex != Vector2.Zero)
				{
					shapeValue["hasNextVertex"] = true;
					VecToJson("nextVertex", chain.NextVertex, shapeValue);
				}
				
				fixtureValue["chain"] = shapeValue;
			}
				break;
			case ShapeType.Polygon:
			{
				PolygonShape poly = (PolygonShape)shape;
				JObject shapeValue = new JObject();
				int vertexCount = poly.Vertices.Count;
				for (int i = 0; i < vertexCount; ++i)
					VecToJson("vertices", poly.Vertices[i], shapeValue, i);
				fixtureValue["polygon"] = shapeValue;
			}
				break;
			default:
				Console.WriteLine("Unknown shape type : " + shape.ShapeType);
				break;
			}
			
			JArray customPropertyValue = WriteCustomPropertiesToJson(fixture);
			if (customPropertyValue.Count > 0)
				fixtureValue["customProperties"] = customPropertyValue;
			
			return fixtureValue;
		}
		
		public void VecToJson(String name, float v, JObject value, int index)
		{
			if (index > -1)
			{
				if (m_useHumanReadableFloats)
				{
					JArray array = (JArray)value[name];
					array[index] = v;
				}
				else
				{
					JArray array = (JArray)value[name];
					if (v == 0)
						array[index] = 0;
					else if (v == 1)
						array[index] = 1;
					else
						array[index] = FloatToHex(v);
				}
			}
			else
				FloatToJson(name, v, value);
		}
		
		public void VecToJson(string name, Vector2 vec, JObject value)
		{
			VecToJson(name, vec, value, -1);
		}
		
		public void VecToJson(string name, Vector2 vec, JObject value, int index)
		{
			
			if (index > -1)
			{
				if (m_useHumanReadableFloats)
				{
					bool alreadyHadArray = value[name] != null;
					JArray arrayX = alreadyHadArray ? (JArray)value[name]["x"] : new JArray();
					JArray arrayY = alreadyHadArray ? (JArray)value[name]["y"] : new JArray();
					arrayX.Add( vec.X);
					arrayY.Add(vec.Y);
					if (!alreadyHadArray)
					{
						JObject subValue = new JObject();
						subValue["x"] = arrayX;
						subValue["y"] = arrayY;
						value[name] = subValue;
					}
				}
				else
				{
					bool alreadyHadArray = value[name] != null;
					JArray arrayX = alreadyHadArray ? (JArray)value[name]["x"] : new JArray();
					JArray arrayY = alreadyHadArray ? (JArray)value[name]["y"] : new JArray();
					if (vec.X == 0)
						arrayX.Add(0);
					else if (vec.X == 1)
						arrayX.Add(1);
					else
						arrayX.Add(FloatToHex(vec.X));
					if (vec.Y == 0)
						arrayY.Add(0);
					else if (vec.Y == 1)
						arrayY.Add(1);
					else
						arrayY.Add(FloatToHex(vec.Y));
					if (!alreadyHadArray)
					{
						JObject subValue = new JObject();
						subValue["x"] = arrayX;
						subValue["y"] = arrayY;
						value[name] = subValue;
					}
				}
			}
			else
			{
				if (vec.X == 0 && vec.Y == 0)
					value[name] = 0;// cut down on file space for common values
				else
				{
					JObject vecValue = new JObject();
					FloatToJson("x", vec.X, vecValue);
					FloatToJson("y", vec.Y, vecValue);
					value[name] = vecValue;
				}
			}
			
		}
		
		protected JArray WriteCustomPropertiesToJson(Object item)
		{
			JArray customPropertiesValue = new JArray();
			
			if (null == item)
				return customPropertiesValue;
			
			Nb2dJsonCustomProperties props = GetCustomPropertiesForItem(item, false);
			
			if (props == null)
				return customPropertiesValue;
			
			
			foreach (var customProp in props.m_customPropertyMap_int)
			{
				KeyValuePair<string, int> pair = (KeyValuePair<string, int>)customProp;
				JObject proValue = new JObject();
				proValue["name"] = pair.Key;
				proValue["int"] = pair.Value;
				customPropertiesValue.Add(proValue);
			}
			
			
			
			foreach (var customProp in props.m_customPropertyMap_float)
			{
				KeyValuePair<string, Double> pair = (KeyValuePair<string, Double>)customProp;
				JObject proValue = new JObject();
				proValue["name"] = pair.Key;
				proValue["float"] = pair.Value;
				customPropertiesValue.Add(proValue);
			}
			
			
			
			
			foreach (var customProp in props.m_customPropertyMap_string)
			{
				KeyValuePair<string, string> pair = (KeyValuePair<string, string>)customProp;
				JObject proValue = new JObject();
				proValue["name"] = pair.Key;
				proValue["string"] = pair.Value;
				customPropertiesValue.Add(proValue);
			}
			
			
			
			foreach (var customProp in props.m_customPropertyMap_vec2)
			{
				KeyValuePair<string, Vector2> pair = (KeyValuePair<string, Vector2>)customProp;
				JObject proValue = new JObject();
				proValue["name"] = pair.Key;
				VecToJson("vec2", pair.Value, proValue);
				customPropertiesValue.Add(proValue);
			}
			
			
			
			foreach (var customProp in props.m_customPropertyMap_bool)
			{
				KeyValuePair<string, bool> pair = (KeyValuePair<string, bool>)customProp;
				JObject proValue = new JObject();
				proValue["name"] = pair.Key;
				proValue["bool"] = pair.Value;
				customPropertiesValue.Add(proValue);
			}
			
			return customPropertiesValue;
		}
		
		public Nb2dJsonCustomProperties GetCustomPropertiesForItem(Object item, bool createIfNotExisting)
		{
			
			if (m_customPropertiesMap.ContainsKey(item))
				return m_customPropertiesMap[item];
			
			if (!createIfNotExisting)
				return null;
			
			Nb2dJsonCustomProperties props = new Nb2dJsonCustomProperties();
			m_customPropertiesMap[item] = props;
			
			return props;
		}
		
		Body lookupBodyFromIndex(int index)
		{
			if (m_indexToBodyMap.ContainsKey(index))
				return m_indexToBodyMap[index];
			else
				return null;
		}
		
		protected int lookupBodyIndex(Body body)
		{
			int? val = m_bodyToIndexMap[body];
			if (null != val)
				return val.Value;
			else
				return -1;
		}
		
		protected int lookupJointIndex(Joint joint)
		{
			int? val = m_jointToIndexMap[joint];
			if (null != val)
				return val.Value;
			else
				return -1;
		}
		
		public String GetImageName(Nb2dJsonImage image)
		{
			if (m_imageToNameMap.ContainsKey(image))
				return m_imageToNameMap[image];
			return null;
		}
		
		public String GetJointName(Joint joint)
		{
			
			if (m_jointToNameMap.ContainsKey(joint))
				return m_jointToNameMap[joint];
			return null;
		}
		
		public string GetBodyName(Body body)
		{
			if (m_bodyToNameMap.ContainsKey(body) )
				return m_bodyToNameMap[body];
			return null;
		}
		
		public String GetFixtureName(Fixture fixture)
		{
			if (m_fixtureToNameMap.ContainsKey(fixture) )
				return m_fixtureToNameMap[fixture];
			return null;
		}
		
		public void FloatToJson(String name, float f, JObject value)
		{
			// cut down on file space for common values
			if (f == 0)
				value.Add(name, 0);
			else if (f == 1)
				value.Add(name, 1);
			else
			{
				if (m_useHumanReadableFloats)
					value.Add(name, f);
				else
					value.Add(name, FloatToHex(f));
			}
		}
		
		// Convert a float argument to a byte array and display it. 
		public string FloatToHex(float argument)
		{
			//no usar todavia
			byte[] byteArray = BitConverter.GetBytes(argument);
			string formatter = "{0,16:E7}{1,20}";
			var res = string.Format(formatter, argument,
			                        BitConverter.ToString(byteArray));
			return res;
		}
		
		
		
		
		
		public Body[] GetBodiesByName(string name)
		{
			List<Body> bodies = new List<Body>();
			foreach (var item in m_bodyToNameMap.Keys)
			{
				if (m_bodyToNameMap[item] == name)
					bodies.Add(item);
			}
			return bodies.ToArray();
		}
		
		
		public Fixture[] GetFixturesByName(String name)
		{
			List<Fixture> fixtures = new List<Fixture>();
			
			foreach (var item in m_fixtureToNameMap.Keys)
			{
				if (m_fixtureToNameMap[item] == name)
					fixtures.Add(item);
			}
			return fixtures.ToArray();
		}
		
		
		
		public Joint[] GetJointsByName(String name)
		{
			List<Joint> joints = new List<Joint>();
			//
			foreach (var item in m_jointToNameMap.Keys)
			{
				if (m_jointToNameMap[item] == name)
					joints.Add(item);
			}
			return joints.ToArray();
		}
		
		
		public Nb2dJsonImage[] GetImagesByName(string name)
		{
			List<Nb2dJsonImage> images = new List<Nb2dJsonImage>();
			foreach (var item in m_imageToNameMap.Keys)
			{
				if (m_imageToNameMap[item] == name)
					images.Add(item);
			}
			return images.ToArray();
		}
		
		
		public Nb2dJsonImage[] GetAllImages()
		{
			return (Nb2dJsonImage[])m_images.ToArray();
		}
		
		public Body GetBodyByName(string name)
		{
			foreach (var item in m_bodyToNameMap.Keys)
			{
				if (m_bodyToNameMap[item] == name)
					return item;
			}
			return null;
			
		}
		
		public Fixture GetFixtureByName(string name)
		{
			foreach (var item in m_fixtureToNameMap.Keys)
			{
				if (m_fixtureToNameMap[item] == name)
					return item;
			}
			return null;
		}
		
		public Joint GetJointByName(String name)
		{
			foreach (var item in m_jointToNameMap.Keys)
			{
				if (m_jointToNameMap[item] == name)
					return item;
			}
			return null;
		}
		
		public Nb2dJsonImage GetImageByName(String name)
		{
			foreach (var item in m_imageToNameMap.Keys)
			{
				if (m_imageToNameMap[item] == name)
					return item;
			}
			return null;
		}
		
		
		
		
		
		
		
		#region salida
		
		public World ReadFromFile(string filename, StringBuilder errorMsg)
		{
			if (null == filename)
				return null;
			
			string str = "";
			try
			{
				System.IO.TextReader readFile = new StreamReader(filename);
				str = readFile.ReadToEnd();
				readFile.Close();
				readFile = null;
				JObject worldValue = JObject.Parse(str);
				return N2b2World(worldValue);
			}
			catch (IOException ex)
			{
				errorMsg.Append("Error reading file: " + filename + ex.Message);
				return null;
			}
		}
		
		public World N2b2World(JObject worldValue)
		{
			World world = new World(jsonToVec("gravity", worldValue));
			Settings.AllowSleep = (bool)worldValue["allowSleep"];
			world.AutoClearForces = (bool)worldValue["autoClearForces"];
			//Settings.EnableWarmstarting = (bool)worldValue["warmStarting"]; //why is const??
			Settings.ContinuousPhysics = (bool)worldValue["continuousPhysics"];
			bool subStepping = worldValue["subStepping"] == null ?  false : (bool)worldValue["subStepping"];
			
			world.EnableSubStepping = subStepping;
			
			readCustomPropertiesFromJson(world, worldValue);
			
			int i = 0;
			JArray bodyValues = (JArray)worldValue["body"];
			if (null != bodyValues)
			{
				int numBodyValues = bodyValues.Count;
				for (i = 0; i < numBodyValues; i++)
				{
					JObject bodyValue = (JObject)bodyValues[i];
					Body body = N2b2Body(world, bodyValue);
					readCustomPropertiesFromJson(body, bodyValue);
					m_bodies.Add(body);
					m_indexToBodyMap.Add(i, body);
				}
			}
			
			// need two passes for joints because gear joints reference other joints
			JArray jointValues = (JArray)worldValue["joint"];
			if (null != jointValues)
			{
				int numJointValues = jointValues.Count;
				for (i = 0; i < numJointValues; i++)
				{
					JObject jointValue = (JObject)jointValues[i];
					if (jointValue["type"].ToString() != "gear")
					{
						Joint joint = j2b2Joint(world, jointValue);
						readCustomPropertiesFromJson(joint, jointValue);
						m_joints.Add(joint);
					}
				}
				for (i = 0; i < numJointValues; i++)
				{
					JObject jointValue = (JObject)jointValues[i];
					if (jointValue["type"].ToString() == "gear")
					{
						Joint joint = j2b2Joint(world, jointValue);
						readCustomPropertiesFromJson(joint, jointValue);
						m_joints.Add(joint);
					}
				}
			}
			i = 0;
			JArray imageValues = (JArray)worldValue["image"];
			if (null != imageValues)
			{
				int numImageValues = imageValues.Count;
				for (i = 0; i < numImageValues; i++)
				{
					JObject imageValue = (JObject)imageValues[i];
					Nb2dJsonImage image = j2b2dJsonImage(imageValue);
					readCustomPropertiesFromJson(image, imageValue);
					m_images.Add(image);
				}
			}
			return world;
		}
		
		
		
		public Body N2b2Body(World world, JObject bodyValue)
		{
			
			var body = BodyFactory.CreateBody(world);
			
			body.BodyType = (BodyType)(int.Parse(bodyValue["type"].ToString()));
			
			
			
			body.Position = jsonToVec("position", bodyValue);
			body.Rotation = jsonToFloat("angle", bodyValue);
			body.LinearVelocity = jsonToVec("linearVelocity", bodyValue);
			body.AngularVelocity = jsonToFloat("angularVelocity", bodyValue);
			body.LinearDamping = jsonToFloat("linearDamping", bodyValue, -1, 0);
			body.AngularDamping = jsonToFloat("angularDamping", bodyValue, -1, 0);
			body.GravityScale = jsonToFloat("gravityScale", bodyValue, -1, 1);
			
			
			body.SleepingAllowed = bodyValue["allowSleep"] == null ? true : (bool)bodyValue["allowSleep"];
			body.Awake = bodyValue["awake"] == null ? false : (bool)bodyValue["awake"];
			body.FixedRotation = bodyValue["fixedRotation"] == null ? false : (bool)bodyValue["fixedRotation"];
			body.IsBullet = bodyValue["bullet"] == null ? false : (bool)bodyValue["bullet"];
			body.Enabled = bodyValue["active"] == null ? true : (bool)bodyValue["active"];
			
			
			
			
			string bodyName = bodyValue["name"] == null ? "" : bodyValue["name"].ToString();
			if ("" != bodyName)
				SetBodyName(body, bodyName);
			
			int i = 0;
			JArray fixtureValues = (JArray)bodyValue["fixture"];
			if (null != fixtureValues)
			{
				int numFixtureValues = fixtureValues.Count;
				for (i = 0; i < numFixtureValues; i++)
				{
					JObject fixtureValue = (JObject)fixtureValues[i];
					Fixture fixture = j2b2Fixture(body, fixtureValue);
					readCustomPropertiesFromJson(fixture, fixtureValue);
				}
			}
			
			// may be necessary if user has overridden mass characteristics
			body.Mass = jsonToFloat("massData-mass", bodyValue);
			body.LocalCenter = jsonToVec("massData-center", bodyValue);
			body.Inertia = jsonToFloat("massData-I", bodyValue);
			
			return body;
		}
		
		
		
		Fixture j2b2Fixture(Body body, JObject fixtureValue)
		{
			
			
			if (null == fixtureValue)
				return null;
			
			
			//Fixture fixtureDef = new Fixture();
			var restitution = jsonToFloat("restitution", fixtureValue);
			var friction = jsonToFloat("friction", fixtureValue);
			var density = jsonToFloat("density", fixtureValue);
			var isSensor = fixtureValue["sensor"] == null ? false : (bool)fixtureValue["sensor"];
			
			
			var categoryBits = fixtureValue["filter-categoryBits"] == null ? 0x0001 : (int)fixtureValue["filter-categoryBits"];
			var maskBits = fixtureValue["filter-maskBits"] == null ? 0xffff : (int)fixtureValue["filter-maskBits"];
			var groupIndex = fixtureValue["filter-groupIndex"] == null ? (short)0 : (short)fixtureValue["filter-groupIndex"];
			
			
			Fixture fixture = null;
			
			
			
			if (null != fixtureValue["circle"])
			{
				JObject circleValue = (JObject)fixtureValue["circle"];
				var radius = jsonToFloat("radius", circleValue);
				var position = jsonToVec("center", circleValue);
				fixture = FixtureFactory.AttachCircle(radius, density, body, position);
			}
			else if (null != fixtureValue["edge"])
			{
				JObject edgeValue = (JObject)fixtureValue["edge"];
				var m_vertex1 = (jsonToVec("vertex1", edgeValue));
				var m_vertex2 = (jsonToVec("vertex2", edgeValue));
				fixture = FixtureFactory.AttachEdge(m_vertex1, m_vertex2, body);
				((EdgeShape)fixture.Shape).HasVertex0 = edgeValue["hasVertex0"] == null ? false : (bool)edgeValue["hasVertex0"];
				((EdgeShape)fixture.Shape).HasVertex3 = edgeValue["hasVertex3"] == null ? false : (bool)edgeValue["hasVertex3"];
				
				if (((EdgeShape)fixture.Shape).HasVertex0)
					((EdgeShape)fixture.Shape).Vertex0 = (jsonToVec("vertex0", edgeValue));
				if (((EdgeShape)fixture.Shape).HasVertex3)
					((EdgeShape)fixture.Shape).Vertex3 = (jsonToVec("vertex3", edgeValue));
				
			}
			else if (null != fixtureValue["loop"])
			{// support old
				// format (r197)
				JObject chainValue = (JObject)fixtureValue["loop"];
				int numVertices = ((JArray)chainValue["x"]).Count;
				Vertices vertices = new Vertices();
				for (int i = 0; i < numVertices; i++)
					vertices.Add(jsonToVec("vertices", chainValue, i));
				fixture = FixtureFactory.AttachChainShape(vertices, body);
				
			}
			else if (null != fixtureValue["chain"])
			{
				JObject chainValue = (JObject)fixtureValue["chain"];
				ChainShape chainShape = new ChainShape();
				int numVertices = ((JArray)chainValue["vertices"]["x"]).Count;
				
				Vertices vertices = new Vertices();
				
				
				for (int i = 0; i < numVertices; i++)
					vertices.Add(jsonToVec("vertices", chainValue, i));
				
				// FPE. See http://www.box2d.org/forum/viewtopic.php?f=4&t=7973&p=35363
				if (vertices[0] == vertices[vertices.Count - 1])
				{
					var vertices2 = new Vertices(numVertices - 1);
					vertices2.AddRange(vertices.GetRange(0, numVertices - 1));
					chainShape.CreateLoop(vertices2);
					fixture = body.CreateFixture(chainShape);
				}
				else
					fixture = FixtureFactory.AttachChainShape(vertices, body);
				
				var fixtureChain = fixture.Shape as ChainShape;
				
				var hasPrevVertex = chainValue["hasPrevVertex"] == null ? false : (bool)chainValue["hasPrevVertex"];
				var hasNextVertex = chainValue["hasNextVertex"] == null ? false : (bool)chainValue["hasNextVertex"];
				if (hasPrevVertex)
					fixtureChain.PrevVertex = (jsonToVec("prevVertex", chainValue));
				if (hasNextVertex)
					fixtureChain.NextVertex = (jsonToVec("nextVertex", chainValue));
				
			}
			else if (null != fixtureValue["polygon"])
			{
				JObject polygonValue = (JObject)fixtureValue["polygon"];
				Vertices vertices = new Vertices();
				int numVertices = ((JArray)polygonValue["vertices"]["x"]).Count;
				if (numVertices > Settings.MaxPolygonVertices)
				{
					Console.WriteLine("Ignoring polygon fixture with too many vertices.");
				}
				else if (numVertices < 2)
				{
					Console.WriteLine("Ignoring polygon fixture less than two vertices.");
				}
				else if (numVertices == 2)
				{
					Console.WriteLine("Creating edge shape instead of polygon with two vertices.");
					var m_vertex1 = (jsonToVec("vertices", polygonValue, 0));
					var m_vertex2 = (jsonToVec("vertices", polygonValue, 1));
					fixture = FixtureFactory.AttachEdge(m_vertex1, m_vertex2, body);
					
				}
				else
				{
					for (int i = 0; i < numVertices; i++)
						vertices.Add(jsonToVec("vertices", polygonValue, i));
					fixture = FixtureFactory.AttachPolygon(vertices, density, body);
				}
			}
			
			String fixtureName = fixtureValue["name"] == null ? "" : fixtureValue["name"].ToString();
			if (fixtureName != "")
			{
				SetFixtureName(fixture, fixtureName);
			}
			
			if (fixture != null)
			{
				fixture.Restitution = restitution;
				fixture.Friction = friction;
				fixture.Shape.Density = density;
				fixture.IsSensor = isSensor;
				fixture.CollisionCategories = (Category)categoryBits;
				fixture.CollidesWith = (Category)maskBits;
				fixture.CollisionGroup = groupIndex;
			}
			
			
			return fixture;
		}
		
		Joint j2b2Joint(World world, JObject jointValue)
		{
			Joint joint = null;
			
			int bodyIndexA = (int)jointValue["bodyA"];
			int bodyIndexB = (int)jointValue["bodyB"];
			if (bodyIndexA >= m_bodies.Count || bodyIndexB >= m_bodies.Count)
				return null;
			
			// set features common to all joints
			var bodyA = m_bodies[bodyIndexA];
			var bodyB = m_bodies[bodyIndexB];
			var collideConnected = jointValue["collideConnected"] == null ? false : (bool)jointValue["collideConnected"];
			
			// keep these in scope after the if/else below
			RevoluteJoint revoluteDef;
			PrismaticJoint prismaticDef;
			DistanceJoint distanceDef;
			PulleyJoint pulleyDef;
			FixedMouseJoint mouseDef;
			GearJoint gearDef;
			WheelJoint wheelDef;
			WeldJoint weldDef;
			FrictionJoint frictionDef;
			RopeJoint ropeDef;
			MotorJoint motorDef;
			
			
			
			Vector2 mouseJointTarget = new Vector2(0, 0);
			string type = jointValue["type"].ToString() == null ? "" : jointValue["type"].ToString();
			if (type == "revolute")
			{
				joint = revoluteDef = JointFactory.CreateRevoluteJoint(world, bodyA, bodyB, jsonToVec("anchorB", jointValue));
				revoluteDef.LocalAnchorA = jsonToVec("anchorA", jointValue);
				revoluteDef.LocalAnchorB = jsonToVec("anchorB", jointValue);
				revoluteDef.ReferenceAngle = jsonToFloat("refAngle", jointValue);
				revoluteDef.LimitEnabled = jointValue["enableLimit"] == null ? false : (bool)jointValue["enableLimit"];
				revoluteDef.LowerLimit = jsonToFloat("lowerLimit", jointValue);
				revoluteDef.UpperLimit = jsonToFloat("upperLimit", jointValue);
				revoluteDef.MotorEnabled = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];
				revoluteDef.MotorSpeed = jsonToFloat("motorSpeed", jointValue);
				revoluteDef.MaxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
			}
			else if (type == "prismatic")
			{
				
				var localAxis = new Vector2();
				
				var localAnchorA = jsonToVec("anchorA", jointValue);
				var localAnchorB = jsonToVec("anchorB", jointValue);
				
				if (jointValue["localAxisA"] != null)
					localAxis = jsonToVec("localAxisA", jointValue);
				else
					localAxis = jsonToVec("localAxis1", jointValue);
				
				
				
				joint = prismaticDef = JointFactory.CreatePrismaticJoint(world, bodyA, bodyB, localAnchorB, localAxis);
				prismaticDef.LocalAnchorA = localAnchorA;
				prismaticDef.ReferenceAngle = jsonToFloat("refAngle", jointValue);
				prismaticDef.LimitEnabled = (bool)jointValue["enableLimit"];
				prismaticDef.LowerLimit = jsonToFloat("lowerLimit", jointValue);
				prismaticDef.UpperLimit = jsonToFloat("upperLimit", jointValue);
				prismaticDef.MotorEnabled = (bool)jointValue["enableMotor"];
				prismaticDef.MotorSpeed = jsonToFloat("motorSpeed", jointValue);
				prismaticDef.MaxMotorForce = jsonToFloat("maxMotorForce", jointValue);
			}
			else if (type == "distance")
			{
				joint = distanceDef = JointFactory.CreateDistanceJoint(world, bodyA, bodyB, jsonToVec("anchorA", jointValue), jsonToVec("anchorB", jointValue));
				distanceDef.LocalAnchorA = (jsonToVec("anchorA", jointValue));
				distanceDef.LocalAnchorB = (jsonToVec("anchorB", jointValue));
				distanceDef.Length = jsonToFloat("length", jointValue);
				distanceDef.Frequency = jsonToFloat("frequency", jointValue);
				distanceDef.DampingRatio = jsonToFloat("dampingRatio", jointValue);
			}
			else if (type == "pulley")
			{
				joint = pulleyDef = JointFactory.CreatePulleyJoint(world, bodyA, bodyB,
				                                                   jsonToVec("groundAnchorA", jointValue), jsonToVec("groundAnchorB", jointValue),
				                                                   jsonToVec("anchorA", jointValue), jsonToVec("anchorB", jointValue),
				                                                   jsonToFloat("ratio", jointValue));
				
				pulleyDef.LengthA = jsonToFloat("lengthA", jointValue);
				pulleyDef.LengthB = jsonToFloat("lengthB", jointValue);
				//pulleyDef.Ratio = jsonToFloat("ratio", jointValue);
			}
			else if (type == "mouse")
			{
				joint = mouseDef = JointFactory.CreateFixedMouseJoint(world, bodyA, jsonToVec("target", jointValue));
				//mouseJointTarget = jsonToVec("target", jointValue);
				//mouseDef.target.set(jsonToVec("anchorB", jointValue));// alter after creating joint
				mouseDef.MaxForce = jsonToFloat("maxForce", jointValue);
				mouseDef.Frequency = jsonToFloat("frequency", jointValue);
				mouseDef.DampingRatio = jsonToFloat("dampingRatio", jointValue);
			}
			// Gear joints are apparently not implemented in JBox2D yet, but
			// when they are, commenting out the following section should work.
			
			else if (type == "gear")
			{
				int jointIndex1 = (int)jointValue["joint1"];
				int jointIndex2 = (int)jointValue["joint2"];
				var joint1 = m_joints[jointIndex1];
				var joint2 = m_joints[jointIndex2];
				var ratio = jsonToFloat("ratio", jointValue);
				
				joint = gearDef = JointFactory.CreateGearJoint(world, joint1, joint2, ratio);
				
			}
			
			// Wheel joints are apparently not implemented in JBox2D yet, but
			// when they are, commenting out the following section should work.
			
			else if (type == "wheel")
			{
				var localAnchorA = jsonToVec("anchorA", jointValue);
				var localAnchorB = (jsonToVec("anchorB", jointValue));
				var localAxisA = (jsonToVec("localAxisA", jointValue));
				var enableMotor = jointValue["enableMotor"] == null ? false : (bool)jointValue["enableMotor"];
				var motorSpeed = jsonToFloat("motorSpeed", jointValue);
				var maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
				var frequencyHz = jsonToFloat("springFrequency", jointValue);
				var dampingRatio = jsonToFloat("springDampingRatio", jointValue);
				
				joint = wheelDef = JointFactory.CreateWheelJoint(world, bodyA, bodyB, localAnchorB, localAxisA);
				
				wheelDef.LocalAnchorA = localAnchorA;
				wheelDef.LocalAnchorB = localAnchorB;
				wheelDef.MotorEnabled = enableMotor;
				wheelDef.MotorSpeed = motorSpeed;
				wheelDef.SpringFrequencyHz = frequencyHz;
				wheelDef.MaxMotorTorque = maxMotorTorque;
				wheelDef.SpringDampingRatio = dampingRatio;
			}
			else if (type == "weld")
			{
				joint = weldDef = JointFactory.CreateWeldJoint(world, bodyA, bodyB, jsonToVec("anchorA", jointValue), jsonToVec("anchorB", jointValue));
                weldDef.LocalAnchorA = jsonToVec("anchorA", jointValue);
                weldDef.LocalAnchorB = jsonToVec("anchorB", jointValue);
                weldDef.FrequencyHz = jsonToFloat("frequency", jointValue);
                weldDef.DampingRatio = jsonToFloat("dampingRatio", jointValue);

			}
			else if (type == "friction")
			{
				joint = frictionDef = JointFactory.CreateFrictionJoint(world, bodyA, bodyB, jsonToVec("anchorA", jointValue));
                frictionDef.LocalAnchorB = jsonToVec("anchorB", jointValue);
				frictionDef.MaxForce = jsonToFloat("maxForce", jointValue);
				frictionDef.MaxTorque = jsonToFloat("maxTorque", jointValue);
			}
			else if (type == "rope")
			{
				joint = ropeDef = new RopeJoint(bodyA, bodyB, jsonToVec("anchorA", jointValue), jsonToVec("anchorB", jointValue));
				world.AddJoint(joint);
				ropeDef.MaxLength = jsonToFloat("maxLength", jointValue);
			}
			
			else if (type == "motor")
			{
				var maxForce = jsonToFloat("maxForce", jointValue);
				var maxMotorTorque = jsonToFloat("maxTorque", jointValue);
				var angularOffset = jsonToFloat("refAngle", jointValue);
				
				joint = motorDef = new MotorJoint(bodyA, bodyB);
                world.AddJoint(joint);
                motorDef.LinearOffset = jsonToVec("anchorA", jointValue);
                motorDef.MaxForce = maxForce;
				motorDef.MaxTorque = maxMotorTorque;
				motorDef.AngularOffset = angularOffset;
			}
			
			
			
			if (null != joint)
			{
				// set features common to all joints
				/*joint.BodyA = bodyA;
				joint.BodyB = bodyB;*/
				joint.CollideConnected = collideConnected;
				
				string jointName = jointValue["name"] == null ? "" : jointValue["name"].ToString();
				if (jointName != "")
				{
					SetJointName(joint, jointName);
				}
			}
			
			
			
			
			return joint;
		}
		
		Nb2dJsonImage j2b2dJsonImage(JObject imageValue)
		{
			Nb2dJsonImage img = new Nb2dJsonImage();
			
			int bodyIndex = imageValue["body"] == null ? -1 : (int)imageValue["body"];
			if (-1 != bodyIndex)
				img.Body = lookupBodyFromIndex(bodyIndex);
			
			String imageName = imageValue["name"] == null ? "" : imageValue["name"].ToString();
			if (imageName != "")
			{
				img.Name = imageName;
				SetImageName(img, imageName);
			}
			
			String fileName = imageValue["file"] == null ? "" : imageValue["file"].ToString();
			if (fileName != "")
				img.File = fileName;
			
			img.Center = jsonToVec("center", imageValue);
			img.Angle = jsonToFloat("angle", imageValue);
			img.Scale = jsonToFloat("scale", imageValue);
			img.Opacity = jsonToFloat("opacity", imageValue);
			img.RenderOrder = jsonToFloat("renderOrder", imageValue);
			
			JArray colorTintArray = (JArray)imageValue["colorTint"];
			if (null != colorTintArray)
			{
				for (int i = 0; i < 4; i++)
				{
					img.ColorTint[i] = (int)colorTintArray[i];
				}
			}
			
			img.Flip = imageValue["flip"] == null ? false : (bool)imageValue["flip"];
			
			img.Filter = imageValue["filter"] == null ? 1 : (int)imageValue["filter"];
			
			img.Corners = new Vector2[4];
			for (int i = 0; i < 4; i++)
				img.Corners[i] = jsonToVec("corners", imageValue, i);
			
			JArray vertexPointerArray = (JArray)imageValue["glVertexPointer"];
			JArray texCoordArray = (JArray)imageValue["glVertexPointer"];
			if (null != vertexPointerArray && null != texCoordArray && vertexPointerArray.Count == texCoordArray.Count)
			{
				int numFloats = vertexPointerArray.Count;
				img.NumPoints = numFloats / 2;
				img.Points = new float[numFloats];
				img.UvCoords = new float[numFloats];
				for (int i = 0; i < numFloats; i++)
				{
					img.Points[i] = jsonToFloat("glVertexPointer", imageValue, i);
					img.UvCoords[i] = jsonToFloat("glTexCoordPointer", imageValue, i);
				}
			}
			
			JArray drawElementsArray = (JArray)imageValue["glDrawElements"];
			if (null != drawElementsArray)
			{
				img.NumIndices = drawElementsArray.Count;
				img.Indices = new short[img.NumIndices];
				for (int i = 0; i < img.NumIndices; i++)
					img.Indices[i] = (short)drawElementsArray[i];
			}
			
			return img;
		}
		
		
		
		
		
		public void SetBodyName(Body body, String name)
		{
			m_bodyToNameMap.Add(body, name);
		}
		
		public void SetFixtureName(Fixture fixture, String name)
		{
			m_fixtureToNameMap.Add(fixture, name);
		}
		
		public void SetJointName(Joint joint, String name)
		{
			m_jointToNameMap.Add(joint, name);
		}
		
		public void SetImageName(Nb2dJsonImage image, String name)
		{
			m_imageToNameMap.Add(image, name);
		}
		
		
		
		protected void readCustomPropertiesFromJson(Body item, JObject value)
		{
			if (null == item)
				return;
			
			if (value["customProperties"] != null)
				return;
			
			int i = 0;
			JArray propValues = (JArray)value["customProperties"];
			if (null != propValues)
			{
				int numPropValues = propValues.Count;
				for (i = 0; i < numPropValues; i++)
				{
					JObject propValue = (JObject)propValues[i];
					string propertyName = propValue["name"].ToString();
					if (propValue["int"] != null)
						SetCustomInt(item, propertyName, (int)propValue["int"]);
					if (propValue["float"] != null)
						SetCustomFloat(item, propertyName, (float)propValue["float"]);
					if (propValue["string"] != null)
						SetCustomString(item, propertyName, propValue["string"].ToString());
					if (propValue["vec2"] != null)
						SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
					if (propValue["bool"] != null)
						SetCustomBool(item, propertyName, (bool)propValue["bool"]);
				}
			}
		}
		
		
		protected void readCustomPropertiesFromJson(Fixture item, JObject value)
		{
			if (null == item)
				return;
			
			if (value["customProperties"] != null)
				return;
			
			int i = 0;
			JArray propValues = (JArray)value["customProperties"];
			if (null != propValues)
			{
				int numPropValues = propValues.Count;
				for (i = 0; i < numPropValues; i++)
				{
					JObject propValue = (JObject)propValues[i];
					String propertyName = propValue["name"].ToString();
					if (propValue["int"] != null)
						SetCustomInt(item, propertyName, (int)propValue["int"]);
					if (propValue["float"] != null)
						SetCustomFloat(item, propertyName, (float)propValue["float"]);
					if (propValue["string"] != null)
						SetCustomString(item, propertyName, propValue["string"].ToString());
					if (propValue["vec2"] != null)
						SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
					if (propValue["bool"] != null)
						SetCustomBool(item, propertyName, (bool)propValue["bool"]);
				}
			}
		}
		
		protected void readCustomPropertiesFromJson(Joint item, JObject value)
		{
			if (null == item)
				return;
			
			if (value["customProperties"] != null)
				return;
			
			int i = 0;
			JArray propValues = (JArray)value["customProperties"];
			if (null != propValues)
			{
				int numPropValues = propValues.Count;
				for (i = 0; i < numPropValues; i++)
				{
					JObject propValue = (JObject)propValues[i];
					String propertyName = propValue["name"].ToString();
					if (propValue["int"] != null)
						SetCustomInt(item, propertyName, (int)propValue["int"]);
					if (propValue["float"] != null)
						SetCustomFloat(item, propertyName, (float)propValue["float"]);
					if (propValue["string"] != null)
						SetCustomString(item, propertyName, propValue["string"].ToString());
					if (propValue["vec2"] != null)
						SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
					if (propValue["bool"] != null)
						SetCustomBool(item, propertyName, (bool)propValue["bool"]);
				}
			}
		}
		
		protected void readCustomPropertiesFromJson(Nb2dJsonImage item, JObject value)
		{
			if (null == item)
				return;
			
			if (value["customProperties"] != null)
				return;
			
			int i = 0;
			JArray propValues = (JArray)value["customProperties"];
			if (null != propValues)
			{
				int numPropValues = propValues.Count;
				for (i = 0; i < numPropValues; i++)
				{
					JObject propValue = (JObject)propValues[i];
					string propertyName = propValue["name"].ToString();
					if (propValue["int"] != null)
						SetCustomInt(item, propertyName, (int)propValue["int"]);
					if (propValue["float"] != null)
						SetCustomFloat(item, propertyName, (float)propValue["float"]);
					if (propValue["string"] != null)
						SetCustomString(item, propertyName, propValue["string"].ToString());
					if (propValue["vec2"] != null)
						SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
					if (propValue["bool"] != null)
						SetCustomBool(item, propertyName, (bool)propValue["bool"]);
				}
			}
		}
		
		protected void readCustomPropertiesFromJson(World item, JObject value)
		{
			if (null == item)
				return;
			
			if (value["customProperties"] != null)
				return;
			
			int i = 0;
			JArray propValues = (JArray)value["customProperties"];
			if (null != propValues)
			{
				int numPropValues = propValues.Count;
				for (i = 0; i < numPropValues; i++)
				{
					JObject propValue = (JObject)propValues[i];
					String propertyName = propValue["name"].ToString();
					if (propValue["int"] != null)
						SetCustomInt(item, propertyName, (int)propValue["int"]);
					if (propValue["float"] != null)
						SetCustomFloat(item, propertyName, (float)propValue["float"]);
					if (propValue["string"] != null)
						SetCustomString(item, propertyName, propValue["string"].ToString());
					if (propValue["vec2"] != null)
						SetCustomVector(item, propertyName, this.jsonToVec("vec2", propValue));
					if (propValue["bool"] != null)
						SetCustomBool(item, propertyName, (bool)propValue["bool"]);
				}
			}
		}
		
		
		
		
		// setCustomXXX
		
		protected void SetCustomInt(Object item, String propertyName, int val)
		{
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
		}
		
		protected void SetCustomFloat(Object item, String propertyName, float val)
		{
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
		}
		
		protected void SetCustomString(Object item, String propertyName, String val)
		{
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
		}
		
		protected void SetCustomVector(Object item, String propertyName, Vector2 val)
		{
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
		}
		
		protected void SetCustomBool(Object item, String propertyName, bool val)
		{
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
		}
		
		
		public void SetCustomInt(Body item, String propertyName, int val)
		{
			m_bodiesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
		}
		
		public void SetCustomFloat(Body item, String propertyName, float val)
		{
			m_bodiesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
		}
		
		public void SetCustomString(Body item, String propertyName, String val)
		{
			m_bodiesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
		}
		
		public void SetCustomVector(Body item, String propertyName, Vector2 val)
		{
			m_bodiesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
		}
		
		public void SetCustomBool(Body item, String propertyName, bool val)
		{
			m_bodiesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
		}
		
		
		public void SetCustomInt(Fixture item, String propertyName, int val)
		{
			m_fixturesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
		}
		
		public void SetCustomFloat(Fixture item, String propertyName, float val)
		{
			m_fixturesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
		}
		
		public void SetCustomString(Fixture item, String propertyName, String val)
		{
			m_fixturesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
		}
		
		public void SetCustomVector(Fixture item, String propertyName, Vector2 val)
		{
			m_fixturesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
		}
		
		public void SetCustomBool(Fixture item, String propertyName, bool val)
		{
			m_fixturesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
		}
		
		
		public void SetCustomInt(Joint item, String propertyName, int val)
		{
			m_jointsWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
		}
		
		public void SetCustomFloat(Joint item, String propertyName, float val)
		{
			m_jointsWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
		}
		
		public void SetCustomString(Joint item, String propertyName, String val)
		{
			m_jointsWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
		}
		
		public void SetCustomVector(Joint item, String propertyName, Vector2 val)
		{
			m_jointsWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
		}
		
		public void SetCustomBool(Joint item, String propertyName, bool val)
		{
			m_jointsWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
		}
		
		
		public void SetCustomInt(Nb2dJsonImage item, String propertyName, int val)
		{
			m_imagesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_int.Add(propertyName, val);
		}
		
		public void SetCustomFloat(Nb2dJsonImage item, String propertyName, float val)
		{
			m_imagesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_float.Add(propertyName, (float)val);
		}
		
		public void SetCustomString(Nb2dJsonImage item, String propertyName, String val)
		{
			m_imagesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_string.Add(propertyName, val);
		}
		
		public void SetCustomVector(Nb2dJsonImage item, String propertyName, Vector2 val)
		{
			m_imagesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_vec2.Add(propertyName, val);
		}
		
		public void SetCustomBool(Nb2dJsonImage item, String propertyName, bool val)
		{
			m_imagesWithCustomProperties.Add(item);
			GetCustomPropertiesForItem(item, true).m_customPropertyMap_bool.Add(propertyName, val);
		}
		
		
		
		
		
		Vector2 jsonToVec(String name, JObject value)
		{
			return jsonToVec(name, value, -1, new Vector2(0, 0));
		}
		
		Vector2 jsonToVec(String name, JObject value, int index)
		{
			return jsonToVec(name, value, index, new Vector2(0, 0));
		}
		
		Vector2 jsonToVec(String name, JObject value, int index, Vector2 defaultValue)
		{
			Vector2 vec = defaultValue;
			
			if (value[name] == null || value[name] is JValue)
				return defaultValue;
			
			if (index > -1)
			{
				JObject vecValue = (JObject)value[name];
				JArray arrayX = (JArray)vecValue["x"];
				JArray arrayY = (JArray)vecValue["y"];
				
				vec.X = (float)arrayX[index];
				
				vec.Y = (float)arrayY[index];
			}
			else
			{
				JObject vecValue = (JObject)value[name];
				if (null == vecValue)
					return defaultValue;
				else if (vecValue["x"] == null) // should be zero vector
					vec.X = vec.Y = 0;
				else
				{
					vec.X = jsonToFloat("x", vecValue);
					vec.Y = jsonToFloat("y", vecValue);
				}
			}
			
			return vec;
		}
		
		float jsonToFloat(String name, JObject value)
		{
			return jsonToFloat(name, value, -1, 0);
		}
		
		float jsonToFloat(String name, JObject value, int index)
		{
			return jsonToFloat(name, value, index, 0);
		}
		
		float jsonToFloat(String name, JObject value, int index, float defaultValue)
		{
			if (value[name] == null)
				return defaultValue;
			
			if (index > -1)
			{
				JArray array = null;
				try
				{
					array = (JArray)value[name];
				}
				catch (Exception e)
				{
				}
				if (null == array)
					return defaultValue;
				Object obj = array[index];
				if (null == obj)
					return defaultValue;
				// else if ( value[name].isString() )
				// return hexToFloat( value[name].asString() );
				else
					return float.Parse(obj.ToString());
			}
			else
			{
				Object obj = value[name];
				if (null == obj)
					return defaultValue;
				// else if ( value[name].isString() )
				// return hexToFloat( value[name].asString() );
				else
					return float.Parse(obj.ToString());
			}
		}
		
		#endregion
		
	}
	
}


