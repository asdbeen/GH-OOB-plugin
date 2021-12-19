using System;
using Rhino;
using Rhino.Commands;
using Rhino.Geometry;
using Rhino.Collections;
using System.Collections.Generic;

namespace OBB
{
    public class OBBSystem
    {
        public int I;
        public System.Collections.Generic.List<GeometryBase> objs;

        public void CheckObjCoPlanarity(List<GeometryBase> objs)
        {
            foreach (var obj in objs)
            {
                if (obj is Rhino.Geometry.Curve)
                {
                    Console.WriteLine("i am a curve");
                }
            }

        }

        public List<Circle> ADDCIRCLE(List<GeometryBase> objs)
        {
            List<Circle> mycirles = new List<Circle>();
            int mycount;

            mycount = objs.Count;

            for(int i = 0; i < mycount; i++)
            {
                Point3d center = new Rhino.Geometry.Point3d(0, 0, 0);
                center = new Rhino.Geometry.Point3d(0, 0, 0);
                const double radius = 10.0;
                mycirles.Add(new Circle(center, radius));
            }
            return mycirles;    
         
        }
    }
}