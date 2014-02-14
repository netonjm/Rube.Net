using Box2D.Collision;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Dynamics;
using Box2D.Dynamics.Joints;
using Box2D.TestBed;
using Cocos2D;
using Farseer2Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RUBE.Cocos2d.Desktop
{
    public class Mouse 
    {

        public int m_textLine;
        float m_angle = 0.0f;

        public const int k_maxContactPoints = 2048;
        public b2Body m_groundBody;
        public b2AABB m_worldAABB;
        public ContactPoint[] m_points = new ContactPoint[k_maxContactPoints];
        public int m_pointCount;
        public DestructionListener m_destructionListener;
        public CCBox2dDraw m_debugDraw;
        public b2World m_world;
        public b2Body m_bomb;
        public b2MouseJoint m_mouseJoint;
        public b2Vec2 m_bombSpawnPoint;
        public bool m_bombSpawning;
        public b2Vec2 m_mouseWorld;
        public int m_stepCount;
        private Settings settings = new Settings();

        public void SetJson(string fullpath)
        {
            Console.WriteLine("Full path is: %s", fullpath);

            Nb2dJson json = new Nb2dJson();
            StringBuilder tmp = new StringBuilder();
            m_world = json.ReadFromFile(fullpath, tmp);

            if (m_world != null)
            {
                Console.WriteLine("Loaded JSON ok");
                m_world.SetDebugDraw(m_debugDraw);

                b2BodyDef bodyDef = new b2BodyDef();
                m_groundBody = m_world.CreateBody(bodyDef);
            }
            else
                Console.WriteLine(tmp); //if this warning bothers you, turn off "Typecheck calls to printf/scanf" in the project build settings

        }

        // This method should undo anything that was done by the loadWorld and afterLoadProcessing
        // methods, and return to a state where loadWorld can safely be called again.
        public void clear()
        {
            if (m_world != null)
            {
                Console.WriteLine("Deleting Box2D world");
                m_world = null;
            }

            if (m_debugDraw != null)
                m_debugDraw = null;

            //m_world = NULL;
            //m_mouseJoint = NULL;
            // m_mouseJointGroundBody = null;
        }

        public Mouse()
        {
            //m_destructionListener = new DestructionListener();
            m_debugDraw = new CCBox2dDraw("fonts/arial-16");

            b2Vec2 gravity = new b2Vec2();
            gravity.Set(500, 500);
           
            m_world = new b2World(gravity);

            
            m_world.SetAllowSleeping(false);
            m_world.SetContinuousPhysics(true);


            m_world.SetDebugDraw(m_debugDraw);
            m_debugDraw.AppendFlags(b2DrawFlags.e_shapeBit | b2DrawFlags.e_aabbBit | b2DrawFlags.e_centerOfMassBit | b2DrawFlags.e_jointBit | b2DrawFlags.e_pairBit);

            m_world.SetContinuousPhysics(true);
            m_world.SetWarmStarting(true);
        }

        public void DrawTitle(int x, int y, string title)
        {
            m_debugDraw.DrawString(x, y, title);
        }

       public void CompleteBombSpawn(b2Vec2 p)
        {
            if (m_bombSpawning == false)
            {
                return;
            }

            const float multiplier = 30.0f;
            b2Vec2 vel = m_bombSpawnPoint - p;
            vel *= multiplier;
            LaunchBomb(m_bombSpawnPoint, vel);
            m_bombSpawning = false;
        }

        public void LaunchBomb(b2Vec2 position, b2Vec2 velocity)
        {
            if (m_bomb != null)
            {
                m_world.DestroyBody(m_bomb);
                m_bomb = null;
            }

            b2BodyDef bd = new b2BodyDef();
            bd.type = b2BodyType.b2_dynamicBody;
            bd.position = position;
            bd.bullet = true;
            m_bomb = m_world.CreateBody(bd);
            m_bomb.LinearVelocity = velocity;

            b2CircleShape circle = new b2CircleShape();
            circle.Radius = 0.3f;

            b2FixtureDef fd = new b2FixtureDef();
            fd.shape = circle;
            fd.density = 20.0f;
            fd.restitution = 0.0f;

            b2Vec2 minV = position - new b2Vec2(0.3f, 0.3f);
            b2Vec2 maxV = position + new b2Vec2(0.3f, 0.3f);

            b2AABB aabb = new b2AABB();
            aabb.LowerBound = minV;
            aabb.UpperBound = maxV;

            m_bomb.CreateFixture(fd);
        }

        public void MouseMove(b2Vec2 p)
        {
            m_mouseWorld = p;

            if (m_mouseJoint != null)
            {
                m_mouseJoint.SetTarget(p);
            }
        }

        public virtual void MouseUp(b2Vec2 p)
        {
            if (m_mouseJoint != null)
            {
                m_world.DestroyJoint(m_mouseJoint);
                m_mouseJoint = null;
            }

            if (m_bombSpawning)
            {
                CompleteBombSpawn(p);
            }
        }

        public virtual bool MouseDown(b2Vec2 p)
        {
            m_mouseWorld = p;

            if (m_mouseJoint != null)
            {
                return false;
            }

            // Make a small box.
            b2AABB aabb = new b2AABB();
            b2Vec2 d = new b2Vec2();
            d.Set(0.001f, 0.001f);
            aabb.LowerBound = p - d;
            aabb.UpperBound = p + d;

            // Query the world for overlapping shapes.
            QueryCallback callback = new QueryCallback(p);
            m_world.QueryAABB(callback, aabb);

            if (callback.m_fixture != null)
            {
                b2Body body = callback.m_fixture.Body;
                b2MouseJointDef md = new b2MouseJointDef();
                md.BodyA = m_groundBody;
                md.BodyB = body;
                md.target = p;
                md.maxForce = 1000.0f * body.Mass;
                m_mouseJoint = (b2MouseJoint)m_world.CreateJoint(md);
                body.SetAwake(true);
                return true;
            }
            return false;
        }

        public void Draw()
        {
         
            m_debugDraw.Begin();

            // DrawTitle(50, 15, "Hola");
            m_debugDraw.DrawString(50, 15, "bodies");
            m_world.DrawDebugData();
            m_debugDraw.End();
        }

        public void Update()
        {
            m_world.Step(1, 8, 1);
            m_world.Gravity.Set(m_world.Gravity.x + 1, m_world.Gravity.y + 1);
            
            //Step();
        }

        public virtual void Step()
        {

            
            float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

            if (settings.pause)
            {
                if (settings.singleStep)
                {
                    settings.singleStep = false;
                }
                else
                {
                    timeStep = 0.0f;
                }

                m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
                m_textLine += 15;
            }

            b2DrawFlags flags = 0;
            if (settings.drawShapes) flags |= b2DrawFlags.e_shapeBit;
            if (settings.drawJoints) flags |= b2DrawFlags.e_jointBit;
            if (settings.drawAABBs) flags |= b2DrawFlags.e_aabbBit;
            if (settings.drawPairs) flags |= b2DrawFlags.e_pairBit;
            if (settings.drawCOMs) flags |= b2DrawFlags.e_centerOfMassBit;
            m_debugDraw.SetFlags(flags);

            m_world.SetWarmStarting(settings.enableWarmStarting > 0);
            m_world.SetContinuousPhysics(settings.enableContinuous > 0);
            m_world.SetSubStepping(settings.enableSubStepping > 0);

            m_pointCount = 0;

            m_world.Step(timeStep, settings.velocityIterations, settings.positionIterations);

            if (timeStep > 0.0f)
            {
                ++m_stepCount;
            }
        }
    }
}
