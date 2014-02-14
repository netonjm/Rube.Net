using System;
using Cocos2D;
using Microsoft.Xna.Framework;
using Box2D.Dynamics;
using Farseer2Json;
using System.Text;
using Box2D.Common;
using Box2D.TestBed;
using Box2D.Collision;
using Box2D.Dynamics.Joints;
using Microsoft.Xna.Framework.Input;

namespace RUBE.Cocos2d.Desktop
{
    public class IntroLayer : CCLayerColor
    {
        
        public CCMenu m_menuLayer;      
        Mouse m_mouse;
        public static float modificator = 4.5f;
        public static float zoommodificator = 0.2f;

        public IntroLayer()
        {
            m_mouse = new Mouse();
            KeyboardEnabled = true;
            TouchEnabled = true;
            AccelerometerEnabled = true;
            AnchorPoint = new CCPoint(0, 0);
            m_mouse.SetJson(GetPath("walker_magic.json"));
            ScheduleUpdate();
        }

        public void goBack(object t)
        {
            UnscheduleUpdate();
            // CCDirector.SharedDirector.ReplaceScene(ExamplesMenuLayer::scene());
        }

        //public void setupMenuLayer()
        //{
        //    CCMenuItem backItem = new CCMenuItemFont("Back", goBack);
        //    CCMenuItem reloadItem = new CCMenuItemFont("Reload", loadWorld);

        //    m_menuLayer = new CCMenu(backItem, reloadItem);
        //    m_menuLayer.AlignItemsHorizontally();

        //    updateAfterOrientationChange();
        //}

        #region MouseDown

    
        #endregion

        void updateAfterOrientationChange()
        {
            CCSize s = CCDirector.SharedDirector.WinSize;
            m_menuLayer.SetPosition(s.Width / 2, s.Height - 20);
        }

        int actual = 0;

      

        public override void Update(float dt)
        {

            MouseState newMouseState = Microsoft.Xna.Framework.Input.Mouse.GetState();
            KeyboardState newKeyboardState = Microsoft.Xna.Framework.Input.Keyboard.GetState();

            if ((newMouseState.ScrollWheelValue > actual) || newKeyboardState.IsKeyDown(Keys.E))
            {
                this.ScaleX += zoommodificator;
                this.ScaleY += zoommodificator;
                actual = newMouseState.ScrollWheelValue;
            }

            if ((newMouseState.ScrollWheelValue < actual) || newKeyboardState.IsKeyDown(Keys.Q))
            {
                this.ScaleX -= zoommodificator;
                this.ScaleY -= zoommodificator;
                actual = newMouseState.ScrollWheelValue;
            }

            if (newKeyboardState.IsKeyDown(Keys.Left) || newKeyboardState.IsKeyDown(Keys.A)) // Press left to pan left.
                PositionX += modificator;

            if (newKeyboardState.IsKeyDown(Keys.Right) || newKeyboardState.IsKeyDown(Keys.D)) // Press right to pan right.
                PositionX -= modificator;

            if (newKeyboardState.IsKeyDown(Keys.Down) || newKeyboardState.IsKeyDown(Keys.S)) // Press down to pan down.
                PositionY += modificator;

            if (newKeyboardState.IsKeyDown(Keys.Up) || newKeyboardState.IsKeyDown(Keys.W)) // Press up to pan up.
                PositionY -= modificator;
            
            base.Update(dt);
            
            m_mouse.Update();

        }

        public override void Draw()
        {

            base.Draw();

            m_mouse.Draw();
           
        }

        bool isMoving = false;
        CCPoint first = new CCPoint();

        public override bool TouchBegan(CCTouch touch)
        {
            CCPoint touchLocation = touch.Location;

            first  = ConvertToNodeSpace(touchLocation);
            isMoving = true;
            
            return true;
        }


        public override void TouchMoved(CCTouch touch)
        {

            if (isMoving)
            {
                CCPoint touchLocation = CCDirector.SharedDirector.ConvertToUi(touch.LocationInView);

               // CCPoint nodePosition = ConvertToNodeSpace(touchLocation);
                Position = new CCPoint(first.X - touchLocation.X, touchLocation.Y - first.Y);
            }
            //m_mouse.MouseMove(new b2Vec2(nodePosition.X, nodePosition.Y));
        }

        public override void TouchEnded(CCTouch touch)
        {
            isMoving = false;
            //CCPoint touchLocation = touch.Location;
            //CCPoint nodePosition = ConvertToNodeSpace(touchLocation);
           // m_test.MouseUp(new b2Vec2(nodePosition.X, nodePosition.Y));
        }


        public static CCScene Scene
        {
            get
            {
                // 'scene' is an autorelease object.
                var scene = new CCScene();

                // 'layer' is an autorelease object.
                var layer = new IntroLayer();

                // add layer as a child to scene
                scene.AddChild(layer);

                // return the scene
                return scene;

            }

        }

        public string GetPath(string file)
        {
            return @"D:\Personal\Mis Documentos\GitHub\RUBE\Rube.Net\Rube.Net\RUBE.Cocos2d.Desktop\Content\" + file;//getFilename()
        }

    }


}


        //private void loadWorld(object t)
        //{
         
        //}

        //void afterLoadProcessing(ref Nb2dJson json)
        //{

        //}

    //    private CCPoint initialWorldOffset()
    //    {
    //         // This function should return the location in pixels to place
    //// the (0,0) point of the physics world. The screen position
    //// will be relative to the bottom left corner of the screen.
    
    ////place (0,0) of physics world at center of bottom edge of screen
    //        CCSize s = CCDirector.SharedDirector.WinSize;
    //return new CCPoint( s.Width/2, 0 );
    //    }

        //private float initialWorldScale()
        //{

        //    CCSize s = CCDirector.SharedDirector.WinSize;
        //    return s.Height / 10; //screen will be 10 physics units high
        //}

        //public CCPoint worldToScreen(b2Vec2 worldPos)
        //{
        //    worldPos *= Scale;
        //    CCPoint layerOffset = Position;
        //    CCPoint p = new CCPoint(worldPos.x + layerOffset.X, worldPos.y + layerOffset.Y);
        //    p.Y = CCDirector.SharedDirector.WinSize.Height - p.Y;
        //    return p;
        //}