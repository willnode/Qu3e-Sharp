using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Qu3e;
using Transform = Qu3e.Transform;
using System.Threading;
using System.IO;

namespace Qu3e_Demo
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            InitPhysics2();
            boxes = new List<Box>();
            scene = InitPhysics(boxes);
        }

        public void Execute()
        {
            var s = new StringBuilder();
            lastT = DateTime.Now;
            while (executed)
            {
                Application.Current.Dispatcher.Invoke(ExecuteVisual);

                var time = DateTime.Now;
                var dt = (time - lastT).TotalSeconds;
                lastT = time;
                var memory = GC.GetTotalMemory(false) / 1024;

                
                try { scene.Step(dt); }
                catch (Exception) { }

                var mem = GC.GetTotalMemory(false) / 1024;
                deltaT = DateTime.Now - time;
                var deltaM = mem - memory;
                steps++;

                capturedGC = deltaM;

                if (deltaM < 0)
                {
                    capturedGCSpeed = -capturedGC / (steps - capturedGCSteps);
                    capturedGCSteps = steps;
                }

            }
        }

        DateTime lastT;
        TimeSpan deltaT;
        int steps;
        long capturedGC;
        long capturedGCSteps;
        long capturedGCSpeed;

        public void ExecuteVisual()
        {

            // Update the view
            for (int i = 0; i < cubes.Count; i++)
            {
                cubes[i].Transform = new MatrixTransform3D(Con(boxes[i].body.GetTransform()));
            }

            timeSpan.Content = "frame: " + steps + ", dt: " + deltaT.Milliseconds + " ms, memory: " +
                (capturedGC > 0 ? "+" + capturedGC : capturedGC.ToString()) + " KB, obj: " + boxes.Count;
            view.InvalidateVisual();
        }

        List<GeometryModel3D> cubes;
        Scene scene;
        List<Box> boxes;

        public Matrix3D Con(Transform v)
        {
            var m = new Matrix3D();

            m.M11 = v.rotation[0][0];
            m.M21 = v.rotation[0][1];
            m.M31 = v.rotation[0][2];
            m.M12 = v.rotation[1][0];
            m.M22 = v.rotation[1][1];
            m.M32 = v.rotation[1][2];
            m.M13 = v.rotation[2][0];
            m.M23 = v.rotation[2][1];
            m.M33 = v.rotation[2][2];
            m.Translate(new Vector3D(v.position[0], v.position[1], v.position[2]));
            return m;
        }

        void InitPhysics2()
        {

            cubes = new List<GeometryModel3D>();

            for (int i = 0; i < 64; i++)
            {
                var c = new GeometryModel3D();
                c.Material = SampleCube.Material;
                c.Geometry = SampleCube.Geometry;
                c.Transform = new TranslateTransform3D(i - 10, 0, 0);
                cubes.Add(c);
                SceneView.Children.Add(c);
            }
            SceneView.Children.Remove(SampleCube);
            view.InvalidateVisual();
        }

        static Scene InitPhysics(List<Box> boxes)
        {
            var scene = new Scene(1 / 100.0, new Vec3(0, -9.8, 0), 10);
            //scene.SetAllowSleep(false);
            

            // Create the floor
            BodyDef bodyDef = new BodyDef();
            Body body = scene.CreateBody(bodyDef);

            ShapeDef ShapeDef = new ShapeDef()
            {
                Restitution = 0.5,
                Friction = 0.7,
                E = new Vec3(25.0, 0.5, 25.0),
            };

            body.AddBox(ShapeDef);

            bodyDef.bodyType = BodyType.eDynamicBody;
            bodyDef.active = true;
            bodyDef.awake = true;
            ShapeDef.E = new Vec3(0.5f, 0.5f, 0.5f);

            var r = new Random();
            for (int i = 0; i < 8; ++i)
            {
                for (int j = 0; j < 8; ++j)
                {
                    bodyDef.position.Set(-5.0f + 1.25f * i, 5.0f, -5.0f + 1.25f * j);
                    bodyDef.axis = new Vec3(0, 0, 1);
                    bodyDef.angle = r.NextDouble() * Math.PI;

                    body = scene.CreateBody(bodyDef);

                    boxes.Add(body.AddBox(ShapeDef));
                }
            }

            return scene;
        }

        bool executed;
        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (!executed)
            {
                executed = true;
                new Thread(new ThreadStart(Execute)).Start();
            }
            else
            {
                BodyDef bodyDef = new BodyDef();
                bodyDef.position.Set(0, 7.0f, 0);
                bodyDef.bodyType = BodyType.eDynamicBody;
                Body body = scene.CreateBody(bodyDef);

                ShapeDef ShapeDef = new ShapeDef();
                boxes.Add(body.AddBox(ShapeDef));

                var c = new GeometryModel3D();
                c.Material = SampleCube.Material;
                c.Geometry = SampleCube.Geometry;
                c.Transform = new TranslateTransform3D(0, 0, 0);
                cubes.Add(c);
                SceneView.Children.Add(c);
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            executed = false;
        }
    }
}
