using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace Qu3e_Demo
{
    public static class GeometrySets
    {
        static Point3D[] cubeVerts = new Point3D[]
        {
             new Point3D (-1,-1,-1),             new Point3D ( 1,-1,-1),
             new Point3D ( 1,-1, 1),             new Point3D (-1,-1, 1),
             new Point3D (-1,-1, 1),             new Point3D ( 1,-1, 1),
             new Point3D ( 1, 1, 1),             new Point3D (-1, 1, 1),
             new Point3D ( 1,-1, 1),             new Point3D ( 1,-1,-1),
             new Point3D ( 1, 1,-1),             new Point3D ( 1, 1, 1),
             new Point3D ( 1, 1, 1),             new Point3D ( 1, 1,-1),
             new Point3D (-1, 1,-1),             new Point3D (-1, 1, 1),
             new Point3D (-1,-1, 1),             new Point3D (-1, 1, 1),
             new Point3D (-1, 1,-1),             new Point3D (-1,-1,-1),
             new Point3D (-1,-1,-1),             new Point3D (-1, 1,-1),
             new Point3D ( 1, 1,-1),             new Point3D ( 1,-1,-1)
        };

        static int[] cubeIndices = new int[] {
             0, 1, 2, 2, 3, 0,
             4, 5, 6, 6, 7, 4,
             8, 9,10,10,11, 8,
            12,13,14,14,15,12,
            16,17,18,18,19,16,
            20,21,22,22,23,20,
        };

        // http://csharphelper.com/blog/2014/10/rotate-a-3d-cube-using-xaml-and-c/
        public static void SetupCube(MeshGeometry3D geom, Vector3D size)
        {
            
            foreach (var p in cubeVerts)
            {
                geom.Positions.Add(new Point3D(size.X * p.X, size.Y * p.Y, size.Z * p.Z));
            }

            foreach (var i in cubeIndices)
            {
                geom.TriangleIndices.Add(i);
            }
        }

        // ----------------------------------------------------------------------------------------

        static void AddTriangle(MeshGeometry3D mesh, Point3D point1, Point3D point2, Point3D point3)
        {
            // Create the points.
            int index1 = mesh.Positions.Count;
            mesh.Positions.Add(point1);
            mesh.Positions.Add(point2);
            mesh.Positions.Add(point3);

            // Create the triangle.
            mesh.TriangleIndices.Add(index1++);
            mesh.TriangleIndices.Add(index1++);
            mesh.TriangleIndices.Add(index1);
        }

        // http://csharphelper.com/blog/2015/04/draw-smooth-spheres-using-wpf-and-c/
        public static void SetupSphere(MeshGeometry3D geom, double radius)
        {
            int num_phi = 8; int num_theta = 16;
            double phi0, theta0;
            double dphi = Math.PI / num_phi;
            double dtheta = 2 * Math.PI / num_theta;

            phi0 = 0;
            double y0 = radius * Math.Cos(phi0);
            double r0 = radius * Math.Sin(phi0);
            for (int i = 0; i < num_phi; i++)
            {
                double phi1 = phi0 + dphi;
                double y1 = radius * Math.Cos(phi1);
                double r1 = radius * Math.Sin(phi1);

                // Point ptAB has phi value A and theta value B.
                // For example, pt01 has phi = phi0 and theta = theta1.
                // Find the points with theta = theta0.
                theta0 = 0;
                Point3D pt00 = new Point3D(
                     r0 * Math.Cos(theta0),
                     y0,
                     r0 * Math.Sin(theta0));
                Point3D pt10 = new Point3D(
                     r1 * Math.Cos(theta0),
                     y1,
                     r1 * Math.Sin(theta0));
                for (int j = 0; j < num_theta; j++)
                {
                    // Find the points with theta = theta1.
                    double theta1 = theta0 + dtheta;
                    Point3D pt01 = new Point3D(
                         r0 * Math.Cos(theta1),
                         y0,
                         r0 * Math.Sin(theta1));
                    Point3D pt11 = new Point3D(
                         r1 * Math.Cos(theta1),
                         y1,
                         r1 * Math.Sin(theta1));

                    // Create the triangles.
                    AddTriangle(geom, pt00, pt11, pt10);
                    AddTriangle(geom, pt00, pt01, pt11);

                    // Move to the next value of theta.
                    theta0 = theta1;
                    pt00 = pt01;
                    pt10 = pt11;
                }

                // Move to the next value of phi.
                phi0 = phi1;
                y0 = y1;
                r0 = r1;
            }
        }

        public static void SetupCapsule(MeshGeometry3D geom, double radius, double extent)
        {
            extent = extent - radius;
            int num_phi = 8; int num_theta = 16;
            double phi0, theta0;
            double dphi = Math.PI / num_phi;
            double dtheta = 2 * Math.PI / num_theta;

            phi0 = 0;
            double y0 = radius * Math.Cos(phi0) + extent * -Math.Sign(phi0 - Math.PI / 2);
            double r0 = radius * Math.Sin(phi0);
            for (int i = 0; i < num_phi; i++)
            {
                double phi1 = phi0 + dphi;
                double y1 = radius * Math.Cos(phi1) + extent * -Math.Sign(phi1 - Math.PI / 2);
                double r1 = radius * Math.Sin(phi1);

                // Point ptAB has phi value A and theta value B.
                // For example, pt01 has phi = phi0 and theta = theta1.
                // Find the points with theta = theta0.
                theta0 = 0;
                Point3D pt00 = new Point3D(
                     r0 * Math.Cos(theta0),
                     y0,
                     r0 * Math.Sin(theta0));
                Point3D pt10 = new Point3D(
                     r1 * Math.Cos(theta0),
                     y1,
                     r1 * Math.Sin(theta0));
                for (int j = 0; j < num_theta; j++)
                {
                    // Find the points with theta = theta1.
                    double theta1 = theta0 + dtheta;
                    Point3D pt01 = new Point3D(
                         r0 * Math.Cos(theta1),
                         y0,
                         r0 * Math.Sin(theta1));
                    Point3D pt11 = new Point3D(
                         r1 * Math.Cos(theta1),
                         y1,
                         r1 * Math.Sin(theta1));

                    // Create the triangles.
                    AddTriangle(geom, pt00, pt11, pt10);
                    AddTriangle(geom, pt00, pt01, pt11);

                    // Move to the next value of theta.
                    theta0 = theta1;
                    pt00 = pt01;
                    pt10 = pt11;
                }

                // Move to the next value of phi.
                phi0 = phi1;
                y0 = y1;
                r0 = r1;
            }
        }

    }
}
