using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;
using Demo;

namespace Rube.TestBed.GUI
{
    public abstract class Screen
    {

        public Color Color { get; set; }

        public abstract void Init(GameMain game);
        public abstract void OnResize();
        public abstract void Update();
        public abstract void Draw();
    }
}
