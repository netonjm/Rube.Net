using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Joints;
using FarseerPhysics.Factories;
using FarseerPhysics.TestBed.Framework;
using Microsoft.Xna.Framework;
using System;
using Rube.TestBed.GUI;
using System.Text;
using Farseer2Json;
using Rube;
using RUBE.MonoGame.WP8;



namespace Rube.TestBed
{
	
	
	public class RubeTestFile : Test
	{
		
		RubeTestFile(World world) : base(world)
		{ 
			
		}
		
		
		public static Test CreateRubeTestFile()
		{ 
			Console.WriteLine(GuiDemo.CurrentRubeFile);
			StringBuilder errorMsg = new StringBuilder();
			Nb2dJson json = new Nb2dJson();
			World world = json.ReadFromFile(GuiDemo.CurrentRubeFile, errorMsg);
			
			var theName = "ball";
			var res = json.GetBodiesByName(theName);
			Console.WriteLine(res);
			
			return new RubeTestFile(world);
		}
	}
	
	
	
	
	public class MotorJointTest : Test
	{
		private MotorJoint _joint;
		private float _time;
		
		MotorJointTest()
		{
			Body ground = BodyFactory.CreateEdge(World, new Vector2(-20, 0), new Vector2(20, 0));
			
			// Define motorized body
			Body body = BodyFactory.CreateRectangle(World, 4, 1, 2, new Vector2(0, 8));
			body.BodyType = BodyType.Dynamic;
			body.Friction = 0.6f;
			
			_joint = new MotorJoint(ground, body);
			_joint.MaxForce = 1000.0f;
			_joint.MaxTorque = 1000.0f;
			
			World.AddJoint(_joint);
		}
		
		public override void Update(GameSettings settings, GameTime gameTime)
		{
			base.Update(settings, gameTime);
			
			
			if (!settings.Pause && settings.Hz > 0.0f)
				_time += 1.0f / settings.Hz;
			
			Vector2 linearOffset = new Vector2();
			linearOffset.X = 6.0f * (float)Math.Sin(2.0f * _time);
			linearOffset.Y = 8.0f + 4.0f * (float)Math.Sin(1.0f * _time);
			
			float angularOffset = 4.0f * _time;
			
			_joint.LinearOffset = linearOffset;
			_joint.AngularOffset = angularOffset;
		}
		
		public static Test Create()
		{
			return new MotorJointTest();
		}
	}
	
	
}
