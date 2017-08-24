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
using Shape = Qu3e.Shape;
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
            InitPhysics();
            InitGeometry();
            InitVisual();

        }

        const int INITIAL_SQ = 0;

        void InitPhysics()
        {

            // Create the floor
            BodyDef bodyDef = new BodyDef();
            Body body = scene.CreateBody(bodyDef);

            ShapeDef ShapeDef = new ShapeDef()
            {
                Type = ShapeType.Box,
                Restitution = 0.5,
                Friction = 0.7,
                Size = new Vec3(50.0, 1.0, 50.0),
            };

            body.AddShape(ShapeDef);

            bodyDef.bodyType = BodyType.eDynamicBody;
            bodyDef.active = true;
            bodyDef.awake = true;
            ShapeDef.Type = ShapeType.Box;
            ShapeDef.Size = new Vec3(1.0, 1.0, 1.0);

            var r = new Random();
            for (int i = 0; i < INITIAL_SQ; ++i)
            {
                for (int j = 0; j < INITIAL_SQ; ++j)
                {
                    bodyDef.position.Set(-5.0f + 1.25f * i, 5.0f, -5.0f + 1.25f * j);
                    bodyDef.axis = new Vec3(0, 0, 1);
                    bodyDef.angle = r.NextDouble() * Math.PI;

                    body = scene.CreateBody(bodyDef);

                    shapes.Add(body.AddShape(ShapeDef));
                }
            }
        }

        void InitGeometry()
        {
            GeometrySets.SetupCube(geometryCube, new Vector3D(0.5, 0.5, 0.5));
            GeometrySets.SetupSphere(geometrySphere, 0.5);
            GeometrySets.SetupCapsule(geometryCapsule, 0.5, 1.5);
        }

        void InitVisual()
        {

            cubes = new List<GeometryModel3D>();

            for (int i = 0; i < INITIAL_SQ * INITIAL_SQ; i++)
            {
                var c = new GeometryModel3D();
                c.Material = materialR;
                c.Geometry = geometryCube;
                c.Transform = new TranslateTransform3D(i - 10, 0, 0);
                cubes.Add(c);
                SceneView.Children.Add(c);
            }
            view.InvalidateVisual();
        }
        
        List<GeometryModel3D> cubes = new List<GeometryModel3D>();
        Scene scene = new Scene(new Vec3(0, -10, 0), 20);
        List<Shape> shapes = new List<Shape>();

        bool executed;

        DateTime lastT;
        TimeSpan deltaT;
        int steps;
        long capturedGC;

        Material materialR = new DiffuseMaterial(new SolidColorBrush(Colors.Red));
        Material materialY = new DiffuseMaterial(new SolidColorBrush(Colors.Yellow));
        Material materialG = new DiffuseMaterial(new SolidColorBrush(Colors.Yellow));
        MeshGeometry3D geometryCube = new MeshGeometry3D();
        MeshGeometry3D geometrySphere = new MeshGeometry3D();
        MeshGeometry3D geometryCapsule = new MeshGeometry3D();

        public void ExecutePhysics(object sender, EventArgs e)
        {
            var s = new StringBuilder();

            var memory = GC.GetTotalMemory(false) / 1024;
            var time = DateTime.Now;
            var dt = (time - lastT).TotalSeconds;

            lastT = time;
                
            try { scene.Step(dt); }
            catch (InvalidOperationException) { }

            deltaT = DateTime.Now - time;
            steps++;
            capturedGC = GC.GetTotalMemory(false) / 1024 - memory;
        }


        public void ExecuteVisual(object sender, EventArgs e)
        {

            // Update the view
            for (int i = 0; i < cubes.Count; i++)
            {
                cubes[i].Transform = new MatrixTransform3D(Con(shapes[i].body.GetTransform()));
            }

            timeSpan.Content = "frame: " + steps + ", dt: " + deltaT.Milliseconds + " ms, memory: " +
                (capturedGC > 0 ? "+" + capturedGC : capturedGC.ToString()) + " KB, obj: " + shapes.Count;
            view.InvalidateVisual();
        }


        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (!executed)
            {
                executed = true;
                CompositionTarget.Rendering += ExecutePhysics;
                CompositionTarget.Rendering += ExecuteVisual;
                lastT = DateTime.Now;
            }
            else
            {
                
                BodyDef bodyDef = new BodyDef();
                bodyDef.position.Set(0, 7.0f, 0);
                bodyDef.angularVelocity = new Vec3(0, 0, 3);
                bodyDef.bodyType = BodyType.eDynamicBody;
                Body body = scene.CreateBody(bodyDef);

                ShapeDef ShapeDef = new ShapeDef();
                ShapeDef.Type = ShapeType.Box;
                ShapeDef.Size = new Vec3(1, 1, 1);
                shapes.Add(body.AddShape(ShapeDef));

                var c = new GeometryModel3D();
                c.Material = materialG;
                c.Geometry = geometryCube;
                c.Transform = new TranslateTransform3D(0, 0, 0);
                cubes.Add(c);
                SceneView.Children.Add(c);
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            executed = false;

        }

        static Matrix3D Con(Transform v)
        {
            var m = new Matrix3D();

            m.M11 = v.rotation[0][0];
            m.M12 = v.rotation[0][1];
            m.M13 = v.rotation[0][2];
            m.M21 = v.rotation[1][0];
            m.M22 = v.rotation[1][1];
            m.M23 = v.rotation[1][2];
            m.M31 = v.rotation[2][0];
            m.M32 = v.rotation[2][1];
            m.M33 = v.rotation[2][2];
            m.Translate(new Vector3D(v.position[0], v.position[1], v.position[2]));
            return m;
        }
    }
}
