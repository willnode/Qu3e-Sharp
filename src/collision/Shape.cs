namespace Qu3e
{
    public abstract class Shape
    {
        public Transform local;

        public Body body;
        public double friction;
        public double restitution;
        public double density;
        public int broadPhaseIndex;
        public object userData;
        public bool sensor;

        public void SetUserdata(object data)
        {
            userData = data;
        }

        public object GetUserdata()
        {
            return userData;
        }

        public void SetSensor(bool isSensor)
        {
            sensor = isSensor;
        }

        public abstract bool TestPoint(Transform tx, Vec3 p);

        public abstract bool Raycast(Transform tx, RaycastData raycast);

        public abstract void ComputeAABB(Transform tx, out AABB aabb);

        public abstract void ComputeMass(out MassData md);

        public abstract void Render(Transform tx, bool awake, Render render);


    }


    public class ShapeDef
    {

        public ShapeDef()
        {
            // Common default values
            Friction = 0.4;
            Restitution = 0.2;
            Density = 1.0;
            Sensor = false;
            Tx = Transform.Identity;
            E = new Vec3(0.5, 0.5, 0.5);
        }

        public void Set(Transform tx, Vec3 size)
        {
            Tx = tx;
            E = size * (0.5);
        }

        public void SetFriction(double friction)
        {
            Friction = friction;
        }
        public void SetRestitution(double restitution)
        {
            Restitution = restitution;
        }
        public void SetDensity(double density)
        {
            Density = density;
        }
        public void SetSensor(bool sensor)
        {
            Sensor = sensor;
        }

        internal Transform Tx;
        internal Vec3 E;

        internal double Friction;
        internal double Restitution;
        internal double Density;
        internal bool Sensor;
    };

}