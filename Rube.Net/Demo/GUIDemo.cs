using System;
using CraftworkGames.CraftworkGui.MonoGame;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Content;
using System.Text;
using Farseer2Json;

namespace Rube
{


	public class GuiDemo
	{


		private GameMain _game;
		public static string CurrentRubeFile { get; set; }
		private MonoGameGuiManager _gui;



		public GuiDemo(GameMain game)
		{
			_game = game;
		}



		public MonoGameGuiManager CreateGui( )
		{
			
			var textureAtlas = new TextureAtlas("Atlas1.png");
            var upRegion = textureAtlas.AddRegion("cog", 48, 0, 47, 47);
            var cogRegion = textureAtlas.AddRegion("up", 0, 0, 47, 47);

			_gui = new MonoGameGuiManager(_game.GraphicsDevice, _game.Content);
			_gui.LoadContent(new GuiContent(textureAtlas, "ExampleFont.fnt", "ExampleFont_0.png"));
			
			var screen = new Screen(800, 480);
			var dockLayout = new DockLayout();
			var gridLayout = new GridLayout(1, 2);
			var leftStackLayout = new StackLayout() { Orientation = Orientation.Horizontal, VerticalAlignment = VerticalAlignment.Bottom };
			var loadRubeFileBtn = CreateButton(cogRegion);
			var dumpRubeFileBtn = CreateButton(upRegion);
						
			dockLayout.Items.Add(new DockItem(gridLayout, DockStyle.Bottom));

            dumpRubeFileBtn.Tag = "dump";
			loadRubeFileBtn.Tag = "load";
			leftStackLayout.Items.Add(loadRubeFileBtn);
			leftStackLayout.Items.Add(dumpRubeFileBtn);
			
			gridLayout.Items.Add(new GridItem(leftStackLayout, 0, 0));
			
			
			screen.Items.Add(dockLayout);
			_gui.Screen = screen;

			return _gui;
		}

		private Button CreateButton(ITextureRegion textureRegion)
		{
			var button = new Button(new VisualStyle(textureRegion)) 
			{ 
				NormalStyle = new VisualStyle(textureRegion),
				HoverStyle = new VisualStyle(textureRegion) 
				{ 
					Scale = new Vector2(1.05f, 1.05f),
				},
				PressedStyle = new VisualStyle(textureRegion)
				{
					Scale = new Vector2(0.95f, 0.95f),
				}
			};
			

			button.Clicked+= HandleClicked;
			return button;
		}

		void HandleClicked (object sender, EventArgs e)
		{
			var button = sender as Button;

			if (button.Tag.ToString() == "load")
			{
				using (System.Windows.Forms.OpenFileDialog op = new System.Windows.Forms.OpenFileDialog())
				{
					op.ShowDialog();
					if (op.FileName.EndsWith(".json"))
					{
						CurrentRubeFile = op.FileName;
						_game.StarTestRube();
					}
				}
			}


            if (button.Tag.ToString() == "dump")
            {
                System.Windows.Forms.SaveFileDialog op = new System.Windows.Forms.SaveFileDialog();
                //op.FileName = @"C:\Users\ivan\Desktop\t1.json";
                op.ShowDialog();
                if (op.FileName.EndsWith(".json"))
                {
                    StringBuilder errorMsg = new StringBuilder();
                    Nb2dJson json = new Nb2dJson();
                    if (!json.WriteToFile(FarseerPhysics.TestBed.Framework.Test.World, op.FileName, 4, errorMsg))// 4-space
                        Console.WriteLine(errorMsg);
                }
            }


		}

	}


}



































